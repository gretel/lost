#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_duckdb.py -- LoRa telemetry collector + live dashboard.

Subscribes to lora_trx CBOR UDP events (all types) and stores them in
a local DuckDB database.  Runs an embedded Bottle web server that serves
a JSON API and a minimal HTML dashboard for live site-survey assessment.

Usage:
    lora_duckdb.py                                    # defaults
    lora_duckdb.py --db data/lora_frames.duckdb       # custom db path
    lora_duckdb.py --dashboard-port 9000              # custom dashboard port
    lora_duckdb.py --no-dashboard                     # collector only

API (primary, machine-readable):
    curl localhost:8099/api/stats?window=4h
    curl localhost:8099/api/frames?window=4h
    curl localhost:8099/api/sf?window=4h
    curl localhost:8099/api/pipeline?window=4h
    curl localhost:8099/api/wideband?window=4h
    curl localhost:8099/api/recent?limit=20

Dashboard (secondary, human-readable):
    open http://localhost:8099/

Offline queries (after stopping collector):
    duckdb data/lora_frames.duckdb -ui

Dependencies: cbor2, duckdb, bottle (all in project venv)
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))

import cbor2
import duckdb

from lora_common import (
    KEEPALIVE_INTERVAL,
    add_logging_args,
    create_udp_subscriber,
    parse_host_port,
    setup_logging,
)

DEFAULT_DB_PATH = "data/lora_frames.duckdb"
DEFAULT_DASHBOARD_PORT = 8099

# Checkpoint every N inserts for WAL durability.
CHECKPOINT_INTERVAL = 100

log = logging.getLogger("gr4.duckdb")


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------

_NEW_FRAME_COLUMNS = [
    ("peak_db", "DOUBLE"),
    ("snr_db_td", "DOUBLE"),
    ("channel_freq", "DOUBLE"),
    ("decode_bw", "DOUBLE"),
    ("cfo_int", "DOUBLE"),
    ("cfo_frac", "DOUBLE"),
    ("sfo_hat", "DOUBLE"),
    ("decode_label", "VARCHAR"),
    ("payload_hash", "UBIGINT"),
    ("device", "VARCHAR"),
    # direction distinguishes RX-decoded frames from TX-emitted frames
    # (lora_trx broadcasts a lora_frame with direction="tx" after each
    # successful transmission; tx rows have NULL for rx-side columns).
    ("direction", "VARCHAR"),
]

_NEW_STATUS_COLUMNS = [
    ("git_rev", "VARCHAR"),
    ("device", "VARCHAR"),
]


def create_tables(con: duckdb.DuckDBPyConnection) -> None:
    """Create all tables if they don't exist, and migrate existing ones."""

    # -- lora_frames (original + new columns) --
    con.execute("""
        CREATE TABLE IF NOT EXISTS lora_frames (
            ts             TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            seq            INTEGER     NOT NULL,
            sf             UTINYINT    NOT NULL,
            bw             UINTEGER    NOT NULL,
            cr             UTINYINT    NOT NULL,
            sync_word      UTINYINT    NOT NULL,
            crc_valid      BOOLEAN     NOT NULL,
            rx_channel     UTINYINT,
            is_downchirp   BOOLEAN     NOT NULL DEFAULT false,
            snr_db         DOUBLE,
            noise_floor_db DOUBLE,
            payload_len    UINTEGER    NOT NULL,
            payload_hex    VARCHAR
        );
    """)
    # Migrate: add columns that may not exist in older databases.
    for col, dtype in _NEW_FRAME_COLUMNS:
        try:
            con.execute(f"ALTER TABLE lora_frames ADD COLUMN {col} {dtype}")
        except duckdb.CatalogException:
            pass  # column already exists

    # -- trx_status (with git_rev + device for provenance) --
    con.execute("""
        CREATE TABLE IF NOT EXISTS trx_status (
            ts              TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            git_rev         VARCHAR,
            device          VARCHAR,
            rx_gain         DOUBLE,
            tx_gain         DOUBLE,
            frames_total    UINTEGER,
            frames_crc_ok   UINTEGER,
            frames_crc_fail UINTEGER,
            rx_overflows    UINTEGER
        );
    """)
    for col, dtype in _NEW_STATUS_COLUMNS:
        try:
            con.execute(f"ALTER TABLE trx_status ADD COLUMN {col} {dtype}")
        except duckdb.CatalogException:
            pass

    # -- multisf_detect --
    con.execute("""
        CREATE TABLE IF NOT EXISTS multisf_detect (
            ts  TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            sf  UTINYINT    NOT NULL,
            bin UINTEGER    NOT NULL
        );
    """)

    # -- multisf_sync --
    con.execute("""
        CREATE TABLE IF NOT EXISTS multisf_sync (
            ts       TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            sf       UTINYINT    NOT NULL,
            cfo_int  INTEGER,
            cfo_frac DOUBLE,
            snr_db   DOUBLE
        );
    """)

    # -- multisf_frame --
    con.execute("""
        CREATE TABLE IF NOT EXISTS multisf_frame (
            ts     TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            sf     UTINYINT    NOT NULL,
            crc_ok BOOLEAN     NOT NULL,
            len    UINTEGER    NOT NULL,
            cr     UTINYINT    NOT NULL,
            snr_db DOUBLE
        );
    """)

    # -- wideband_sweep --
    con.execute("""
        CREATE TABLE IF NOT EXISTS wideband_sweep (
            ts           TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            sweep        UINTEGER    NOT NULL,
            tainted      BOOLEAN     NOT NULL,
            overflows    UINTEGER    NOT NULL,
            zero_calls   UINTEGER    NOT NULL,
            total_calls  UINTEGER    NOT NULL,
            duration_ms  UINTEGER    NOT NULL,
            n_snapshots  UINTEGER    NOT NULL,
            n_hot        UINTEGER    NOT NULL,
            n_active     UINTEGER    NOT NULL,
            max_slots    UINTEGER    NOT NULL
        );
    """)

    # -- wideband_slot --
    con.execute("""
        CREATE TABLE IF NOT EXISTS wideband_slot (
            ts      TIMESTAMPTZ NOT NULL DEFAULT current_timestamp,
            action  VARCHAR     NOT NULL,
            slot    UINTEGER    NOT NULL,
            channel UINTEGER    NOT NULL,
            freq    DOUBLE      NOT NULL,
            bw      UINTEGER    NOT NULL
        );
    """)


# ---------------------------------------------------------------------------
# Insert functions
# ---------------------------------------------------------------------------

_INSERT_FRAME_SQL = """
    INSERT INTO lora_frames (
        seq, direction, sf, bw, cr, sync_word, crc_valid, rx_channel,
        is_downchirp, snr_db, noise_floor_db, payload_len, payload_hex,
        peak_db, snr_db_td, channel_freq, decode_bw,
        cfo_int, cfo_frac, sfo_hat, decode_label, payload_hash, device
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
"""


def insert_frame(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    """Insert a lora_frame CBOR message into DuckDB.

    Reads frame identity from `msg.carrier` (sync_word/sf/bw/cr) and DSP
    state from `msg.phy` (snr_db/cfo/…).  TX frames (direction="tx") have
    no phy sub-map — those DSP columns end up NULL.
    """
    payload = msg.get("payload", b"")
    carrier = msg.get("carrier", {})
    phy = msg.get("phy", {})
    direction = msg.get("direction", "rx")
    con.execute(
        _INSERT_FRAME_SQL,
        [
            msg.get("seq", 0),
            direction,
            carrier.get("sf", 0),
            carrier.get("bw", 0),
            carrier.get("cr", 0),
            carrier.get("sync_word", 0),
            msg.get("crc_valid", False),
            msg.get("rx_channel"),
            msg.get("is_downchirp", False),
            phy.get("snr_db"),
            phy.get("noise_floor_db"),
            len(payload),
            payload.hex() if payload else None,
            phy.get("peak_db"),
            phy.get("snr_db_td"),
            phy.get("channel_freq"),
            phy.get("decode_bw"),
            phy.get("cfo_int"),
            phy.get("cfo_frac"),
            phy.get("sfo_hat"),
            msg.get("decode_label"),
            msg.get("payload_hash"),
            msg.get("device"),
        ],
    )


def insert_status(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    """Insert a status heartbeat into DuckDB."""
    phy = msg.get("phy", {})
    frames = msg.get("frames", {})
    con.execute(
        """INSERT INTO trx_status
           (git_rev, device, rx_gain, tx_gain, frames_total, frames_crc_ok,
            frames_crc_fail, rx_overflows)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?);""",
        [
            msg.get("git_rev"),
            msg.get("device"),
            phy.get("rx_gain"),
            phy.get("tx_gain"),
            frames.get("total"),
            frames.get("crc_ok"),
            frames.get("crc_fail"),
            msg.get("rx_overflows"),
        ],
    )


def insert_multisf_detect(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    con.execute(
        "INSERT INTO multisf_detect (sf, bin) VALUES (?, ?);",
        [msg.get("sf", 0), msg.get("bin", 0)],
    )


def insert_multisf_sync(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    con.execute(
        "INSERT INTO multisf_sync (sf, cfo_int, cfo_frac, snr_db) VALUES (?, ?, ?, ?);",
        [msg.get("sf", 0), msg.get("cfo_int"), msg.get("cfo_frac"), msg.get("snr_db")],
    )


def insert_multisf_frame(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    con.execute(
        "INSERT INTO multisf_frame (sf, crc_ok, len, cr, snr_db) VALUES (?, ?, ?, ?, ?);",
        [
            msg.get("sf", 0),
            msg.get("crc_ok", False),
            msg.get("len", 0),
            msg.get("cr", 0),
            msg.get("snr_db"),
        ],
    )


def insert_wideband_sweep(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    con.execute(
        """INSERT INTO wideband_sweep
           (sweep, tainted, overflows, zero_calls, total_calls, duration_ms,
            n_snapshots, n_hot, n_active, max_slots)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);""",
        [
            msg.get("sweep", 0),
            msg.get("tainted", False),
            msg.get("overflows", 0),
            msg.get("zero_calls", 0),
            msg.get("total_calls", 0),
            msg.get("duration_ms", 0),
            msg.get("n_snapshots", 0),
            msg.get("n_hot", 0),
            msg.get("n_active", 0),
            msg.get("max_slots", 0),
        ],
    )


def insert_wideband_slot(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    con.execute(
        """INSERT INTO wideband_slot
           (action, slot, channel, freq, bw)
           VALUES (?, ?, ?, ?, ?);""",
        [
            msg.get("action", ""),
            msg.get("slot", 0),
            msg.get("channel", 0),
            msg.get("freq", 0.0),
            msg.get("bw", 0),
        ],
    )


# Dispatch table: CBOR type -> insert function
_INSERTERS: dict[str, Any] = {
    "lora_frame": insert_frame,
    "status": insert_status,
    "multisf_detect": insert_multisf_detect,
    "multisf_sync": insert_multisf_sync,
    "multisf_frame": insert_multisf_frame,
    "wideband_sweep": insert_wideband_sweep,
    "wideband_slot": insert_wideband_slot,
}


# ---------------------------------------------------------------------------
# Dashboard -- JSON API + HTML
# ---------------------------------------------------------------------------

# Time window -> (SQL interval string, bucket interval string)
_WINDOWS: dict[str, tuple[str, str]] = {
    "1h": ("1 hour", "1 minute"),
    "4h": ("4 hours", "5 minutes"),
    "12h": ("12 hours", "15 minutes"),
    "24h": ("24 hours", "30 minutes"),
}
# "all" is handled specially (no WHERE clause, 1 hour buckets).


def _window_clause(window: str) -> str:
    """Return a SQL WHERE clause fragment for the time window."""
    if window == "all":
        return ""
    interval = _WINDOWS.get(window, _WINDOWS["4h"])[0]
    return f"WHERE ts >= now() - INTERVAL '{interval}'"


def _bucket_interval(window: str) -> str:
    """Return the bucket interval string for the time window."""
    if window == "all":
        return "1 hour"
    return _WINDOWS.get(window, _WINDOWS["4h"])[1]


def _json_serial(obj: Any) -> Any:
    """JSON serializer for types not handled by default."""
    if isinstance(obj, datetime):
        return obj.isoformat()
    raise TypeError(f"Type {type(obj)} not serializable")


def _query_json(con: duckdb.DuckDBPyConnection, sql: str) -> list[dict[str, Any]]:
    """Execute SQL and return result as a list of dicts."""
    result = con.execute(sql)
    cols = [desc[0] for desc in result.description]
    rows = result.fetchall()
    return [dict(zip(cols, row)) for row in rows]


def _safe_query(con: duckdb.DuckDBPyConnection, sql: str) -> list[dict[str, Any]]:
    """Query with error handling -- returns empty list on failure."""
    try:
        return _query_json(con, sql)
    except Exception as exc:
        log.debug("dashboard query failed: %s", exc)
        return []


def create_dashboard_app(con: duckdb.DuckDBPyConnection) -> Any:
    """Create and return the Bottle WSGI app.

    Uses con.cursor() to create a thread-local DuckDB handle for the
    dashboard thread.  DuckDB supports concurrent reads + appends within
    a single process via MVCC, but each thread must use its own cursor.
    See: https://duckdb.org/docs/current/guides/python/multiple_threads
    """
    import bottle

    app = bottle.Bottle()
    # Thread-local cursor -- dashboard reads never conflict with
    # collector appends (DuckDB MVCC, optimistic concurrency).
    local_con = con.cursor()

    # Suppress Bottle's default logging (it uses warnings module).
    logging.getLogger("bottle").setLevel(logging.WARNING)

    @app.get("/api/stats")
    def api_stats() -> str:
        window = bottle.request.params.get("window", "4h")
        wc = _window_clause(window)
        rows = _safe_query(
            local_con,
            f"""
            SELECT
                count(*)                              AS total_frames,
                coalesce(sum(crc_valid::int), 0)      AS crc_ok,
                count(*) - coalesce(sum(crc_valid::int), 0) AS crc_fail,
                round(coalesce(avg(crc_valid::int), 0) * 100, 1) AS crc_pct,
                round(avg(snr_db), 1)                 AS avg_snr,
                round(min(snr_db), 1)                 AS min_snr,
                round(max(snr_db), 1)                 AS max_snr,
                round(avg(noise_floor_db), 1)         AS avg_noise_floor,
                round(avg(peak_db), 1)                AS avg_peak,
                round(avg(snr_db_td), 1)              AS avg_snr_td,
                round(avg(cfo_frac), 3)               AS avg_cfo_frac,
                round(avg(abs(sfo_hat)), 6)           AS avg_abs_sfo
            FROM lora_frames {wc}
        """,
        )
        bottle.response.content_type = "application/json"
        return json.dumps(rows[0] if rows else {}, default=_json_serial)

    @app.get("/api/frames")
    def api_frames() -> str:
        window = bottle.request.params.get("window", "4h")
        wc = _window_clause(window)
        bucket = _bucket_interval(window)
        rows = _safe_query(
            local_con,
            f"""
            SELECT
                strftime(time_bucket(INTERVAL '{bucket}', ts),
                         '%Y-%m-%dT%H:%M:%S')         AS t,
                count(*)                               AS frames,
                round(avg(crc_valid::int) * 100, 1)    AS crc_pct,
                round(avg(snr_db), 1)                  AS avg_snr,
                round(avg(noise_floor_db), 1)          AS avg_noise_floor
            FROM lora_frames {wc}
            GROUP BY t ORDER BY t
        """,
        )
        bottle.response.content_type = "application/json"
        return json.dumps(rows, default=_json_serial)

    @app.get("/api/sf")
    def api_sf() -> str:
        window = bottle.request.params.get("window", "4h")
        wc = _window_clause(window)
        rows = _safe_query(
            local_con,
            f"""
            SELECT
                sf,
                count(*)                              AS frames,
                coalesce(sum(crc_valid::int), 0)      AS crc_ok,
                round(coalesce(avg(crc_valid::int), 0) * 100, 1) AS crc_pct,
                round(avg(snr_db), 1)                 AS avg_snr,
                round(min(snr_db), 1)                 AS min_snr,
                round(max(snr_db), 1)                 AS max_snr,
                round(avg(cfo_frac), 3)               AS avg_cfo_frac,
                round(avg(abs(sfo_hat)), 6)           AS avg_abs_sfo
            FROM lora_frames {wc}
            GROUP BY sf ORDER BY sf
        """,
        )
        bottle.response.content_type = "application/json"
        return json.dumps(rows, default=_json_serial)

    @app.get("/api/pipeline")
    def api_pipeline() -> str:
        window = bottle.request.params.get("window", "4h")
        wc = _window_clause(window)

        # Totals
        detect = _safe_query(
            local_con, f"SELECT count(*) AS n FROM multisf_detect {wc}"
        )
        sync = _safe_query(local_con, f"SELECT count(*) AS n FROM multisf_sync {wc}")
        frame = _safe_query(local_con, f"SELECT count(*) AS n FROM multisf_frame {wc}")
        n_detect = detect[0]["n"] if detect else 0
        n_sync = sync[0]["n"] if sync else 0
        n_frame = frame[0]["n"] if frame else 0

        # Per-SF funnel
        per_sf_detect = {
            r["sf"]: r["n"]
            for r in _safe_query(
                local_con,
                f"SELECT sf, count(*) AS n FROM multisf_detect {wc} GROUP BY sf",
            )
        }
        per_sf_sync = {
            r["sf"]: r
            for r in _safe_query(
                local_con,
                f"""SELECT sf, count(*) AS n,
                           round(avg(snr_db), 1) AS avg_snr,
                           round(avg(cfo_frac), 3) AS avg_cfo_frac
                    FROM multisf_sync {wc} GROUP BY sf""",
            )
        }
        per_sf_frame = {
            r["sf"]: r
            for r in _safe_query(
                local_con,
                f"""SELECT sf, count(*) AS n,
                           coalesce(sum(crc_ok::int), 0) AS crc_ok,
                           round(coalesce(avg(crc_ok::int), 0) * 100, 1) AS crc_pct
                    FROM multisf_frame {wc} GROUP BY sf""",
            )
        }

        all_sfs = sorted(set(per_sf_detect) | set(per_sf_sync) | set(per_sf_frame))
        per_sf = []
        for sf in all_sfs:
            nd = per_sf_detect.get(sf, 0)
            ns_row = per_sf_sync.get(sf, {})
            ns = ns_row.get("n", 0) if ns_row else 0
            nf_row = per_sf_frame.get(sf, {})
            nf = nf_row.get("n", 0) if nf_row else 0
            per_sf.append(
                {
                    "sf": sf,
                    "detect": nd,
                    "sync": ns,
                    "frame": nf,
                    "detect_to_sync_pct": round(ns / nd * 100, 1) if nd else None,
                    "sync_to_frame_pct": round(nf / ns * 100, 1) if ns else None,
                    "avg_sync_snr": ns_row.get("avg_snr") if ns_row else None,
                    "avg_cfo_frac": ns_row.get("avg_cfo_frac") if ns_row else None,
                    "crc_pct": nf_row.get("crc_pct") if nf_row else None,
                }
            )

        result = {
            "detect": n_detect,
            "sync": n_sync,
            "frame": n_frame,
            "detect_to_sync_pct": round(n_sync / n_detect * 100, 1)
            if n_detect
            else None,
            "sync_to_frame_pct": round(n_frame / n_sync * 100, 1) if n_sync else None,
            "detect_to_frame_pct": round(n_frame / n_detect * 100, 1)
            if n_detect
            else None,
            "per_sf": per_sf,
        }
        bottle.response.content_type = "application/json"
        return json.dumps(result, default=_json_serial)

    @app.get("/api/wideband")
    def api_wideband() -> str:
        window = bottle.request.params.get("window", "4h")
        wc = _window_clause(window)
        rows = _safe_query(
            local_con,
            f"""
            SELECT
                count(*)                                AS sweeps,
                coalesce(sum(tainted::int), 0)          AS tainted,
                round(coalesce(avg(tainted::int), 0) * 100, 1) AS tainted_pct,
                round(avg(duration_ms), 0)              AS avg_duration_ms,
                round(avg(overflows), 1)                AS avg_overflows,
                coalesce(sum(overflows), 0)             AS total_overflows,
                round(avg(n_hot), 1)                    AS avg_hot,
                round(avg(n_active), 1)                 AS avg_active,
                max(max_slots)                          AS max_slots
            FROM wideband_sweep {wc}
        """,
        )
        bottle.response.content_type = "application/json"
        return json.dumps(rows[0] if rows else {}, default=_json_serial)

    @app.get("/api/recent")
    def api_recent() -> str:
        try:
            limit = int(bottle.request.params.get("limit", "20"))
        except ValueError:
            limit = 20
        limit = max(1, min(limit, 200))
        rows = _safe_query(
            local_con,
            f"""
            SELECT strftime(ts, '%Y-%m-%dT%H:%M:%S.') || lpad(extract('ms' FROM ts)::int::varchar, 3, '0') AS ts,
                   seq, sf, bw, cr, sync_word, crc_valid,
                   snr_db, noise_floor_db, peak_db, snr_db_td,
                   channel_freq, decode_bw, cfo_int, cfo_frac, sfo_hat,
                   payload_len, decode_label, is_downchirp, device
            FROM lora_frames ORDER BY ts DESC LIMIT {limit}
        """,
        )
        bottle.response.content_type = "application/json"
        return json.dumps(rows, default=_json_serial)

    @app.get("/")
    def index() -> str:
        bottle.response.content_type = "text/html; charset=utf-8"
        return _DASHBOARD_HTML

    return app


def start_dashboard(con: duckdb.DuckDBPyConnection, port: int) -> None:
    """Start the Bottle dashboard in a daemon thread."""
    app = create_dashboard_app(con)

    def _serve() -> None:
        import bottle

        # Use the quiet WSGIRef server -- no access logs.
        bottle.run(app, host="0.0.0.0", port=port, quiet=True)

    t = threading.Thread(target=_serve, daemon=True, name="dashboard")
    t.start()
    log.info("dashboard: http://localhost:%d/", port)


# ---------------------------------------------------------------------------
# HTML dashboard (single self-contained page)
# ---------------------------------------------------------------------------

_DASHBOARD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LoRa Open Source Transceiver</title>
<style>
  :root { --bg: #0d0d14; --card: #12121f; --border: #1e1e35; --text: #c8c8d4;
          --ok: #00ffaa; --warn: #ffaa00; --fail: #ff3366; --muted: #606080;
          --accent: #7b68ee; }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, system-ui, 'Segoe UI', Helvetica, Arial, sans-serif;
         font-size: 13px; background: var(--bg); color: var(--text); padding: 10px; }
  h1 { font-size: 15px; font-weight: 600; color: var(--accent); }
  h2 { font-size: 11px; margin-bottom: 4px; color: var(--muted); text-transform: uppercase;
       letter-spacing: 1px; font-weight: 500; }
  .g4 { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; margin-bottom: 6px; }
  .g4 .span2 { grid-column: span 2; }
  .g4 .span3 { grid-column: span 3; }
  .g4 .span4 { grid-column: span 4; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 4px;
          padding: 8px 10px; display: flex; flex-direction: column;
          min-width: 0; overflow: hidden; }
  .card > *:last-child { flex: 1; }
  .top-bar { display: flex; justify-content: space-between; align-items: center;
             margin-bottom: 8px; flex-wrap: wrap; gap: 6px; }
  .window-sel button { background: var(--card); border: 1px solid var(--border);
                       color: var(--text); padding: 2px 8px; border-radius: 3px;
                       cursor: pointer; font-family: inherit; font-size: 11px;
                       margin-left: 2px; }
  .window-sel button.active { background: var(--accent); color: #fff;
                              border-color: var(--accent); font-weight: 600; }
  table { width: 100%; border-collapse: collapse; }
  th, td { text-align: right; padding: 3px 6px; border-bottom: 1px solid var(--border);
           overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  th { color: var(--muted); font-weight: 500; font-size: 11px;
       border-bottom: 2px solid var(--border); }
  td:first-child, th:first-child { text-align: left; }
  .kv td:first-child { color: var(--muted); font-size: 12px; white-space: nowrap; }
  .kv td:last-child { font-family: 'SF Mono', 'Fira Code', ui-monospace, monospace;
                      font-size: 13px; font-weight: 600; color: #e0e0ff; }
  .good { color: var(--ok); }  .warn { color: var(--warn); }  .bad { color: var(--fail); }
  .trend-row { display: flex; align-items: flex-end; gap: 1px; height: 36px; flex: 1; }
  .trend-bar { flex: 1; background: var(--ok); border-radius: 1px 1px 0 0; min-width: 2px; }
  .trend-bar.warn { background: var(--warn); }
  .trend-bar.fail { background: var(--fail); }
  #status { font-size: 11px; color: var(--muted); }
  .mono { font-family: 'SF Mono', 'Fira Code', ui-monospace, monospace; }
  th.sortable { cursor: pointer; user-select: none; }
  th.sortable:hover { color: var(--text); }
  th.sort-asc::after { content: ' \\25B2'; font-size: 9px; }
  th.sort-desc::after { content: ' \\25BC'; font-size: 9px; }
  .funnel-row { display: flex; align-items: center; gap: 6px; margin-bottom: 3px; }
  .funnel-label { width: 36px; font-size: 12px; color: var(--muted); text-align: right; flex-shrink: 0; }
  .funnel-bars { position: relative; height: 12px; width: 100%; max-width: 100%;
                 background: var(--bg); border-radius: 2px; }
  .funnel-bars .fb { position: absolute; top: 0; left: 0; height: 100%; border-radius: 2px; }
  .fb-detect { background: #1e1e35; }
  .fb-sync   { background: var(--warn); opacity: 0.85; }
  .fb-frame  { background: var(--ok); }
  .funnel-stats { font-size: 11px; color: var(--muted); flex-shrink: 0; white-space: nowrap; }
  .funnel-stats .mono { color: var(--text); }
</style>
</head>
<body>
<div class="top-bar">
  <h1>LoRa Open Source Transceiver</h1>
  <div>
    <span id="status">loading...</span>
    <span class="window-sel">
      <button onclick="setWindow('1h')">1h</button>
      <button onclick="setWindow('4h')" class="active">4h</button>
      <button onclick="setWindow('12h')">12h</button>
      <button onclick="setWindow('24h')">24h</button>
      <button onclick="setWindow('all')">all</button>
    </span>
  </div>
</div>

<div class="g4">
  <div class="card">
    <h2>Overview</h2>
    <table class="kv" id="overview"><tr><td colspan="2">-</td></tr></table>
  </div>
  <div class="card span3">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px">
      <h2 style="margin-bottom:0">Per-SF Decoder Pipeline</h2>
      <span style="font-size:11px;color:var(--muted);white-space:nowrap"><span style="display:inline-block;width:8px;height:8px;background:#1e1e35;border-radius:1px;vertical-align:middle"></span> detect <span style="display:inline-block;width:8px;height:8px;background:var(--warn);border-radius:1px;vertical-align:middle"></span> sync <span style="display:inline-block;width:8px;height:8px;background:var(--ok);border-radius:1px;vertical-align:middle"></span> frame</span>
    </div>
    <div id="sf-pipeline" style="min-width:0;overflow-x:auto">-</div>
  </div>
</div>

<div class="g4">
  <div class="card"><h2>Frame Rate</h2><div id="trend-frames" class="trend-row"></div></div>
  <div class="card"><h2>CRC %</h2><div id="trend-crc" class="trend-row"></div></div>
  <div class="card"><h2>Avg SNR (dB)</h2><div id="trend-snr" class="trend-row"></div></div>
  <div class="card"><h2>Noise Floor (dBFS)</h2><div id="trend-noise" class="trend-row"></div></div>
</div>

<div class="g4">
  <div class="card span4">
    <h2>Recent Frames</h2>
    <div style="overflow-x:auto"><table id="recent-table"><tr><td>-</td></tr></table></div>
  </div>
</div>

<script>
let W = '4h';
let recentData = [];
let sortCol = null, sortDir = 1; // 1=asc, -1=desc

function setWindow(w) {
  W = w;
  document.querySelectorAll('.window-sel button').forEach(b =>
    b.classList.toggle('active', b.textContent === w));
  refresh();
}

function q(url) { return fetch(url).then(r => r.json()).catch(() => null); }

function fmt(v, d) {
  if (v === null || v === undefined) return '-';
  if (typeof v === 'number') return d !== undefined ? v.toFixed(d) : String(v);
  return String(v);
}

function crcClass(pct) {
  if (pct === null || pct === undefined) return '';
  return pct >= 90 ? 'good' : pct >= 70 ? 'warn' : 'bad';
}

function kvRow(label, value, cls) {
  return `<tr><td>${label}</td><td class="${cls||''}">${value}</td></tr>`;
}

function renderOverview(d) {
  const el = document.getElementById('overview');
  if (!d || !d.total_frames) { el.innerHTML = '<tr><td colspan="2">no data</td></tr>'; return; }
  el.innerHTML =
    kvRow('Frames', d.total_frames) +
    kvRow('CRC', fmt(d.crc_pct,1)+'%', crcClass(d.crc_pct)) +
    kvRow('Avg SNR', fmt(d.avg_snr,1)+' dB') +
    kvRow('SNR range', fmt(d.min_snr,1)+' / '+fmt(d.max_snr,1)+' dB') +
    kvRow('Noise floor', fmt(d.avg_noise_floor,1)+' dBFS') +
    kvRow('CFO frac', fmt(d.avg_cfo_frac,3)) +
    kvRow('|SFO|', fmt(d.avg_abs_sfo,5));
}

function renderSfPipeline(sfData, pipeData) {
  const el = document.getElementById('sf-pipeline');
  if ((!sfData || !sfData.length) && (!pipeData || !pipeData.detect)) {
    el.textContent = 'no data'; return;
  }
  const p = v => v !== null && v !== undefined ? fmt(v,1)+'%' : '-';

  // Build per-SF merged rows: combine /api/sf data with /api/pipeline per_sf
  const pipeMap = {};
  if (pipeData && pipeData.per_sf) {
    pipeData.per_sf.forEach(r => { pipeMap[r.sf] = r; });
  }

  // Table with funnel bar column
  let h = '';
  const maxD = pipeData && pipeData.per_sf ? Math.max(...pipeData.per_sf.flatMap(r => [r.detect||0, r.sync||0, r.frame||0]), 1) : 1;
  h += '<table style="table-layout:fixed;width:100%">' +
       '<colgroup><col style="width:5%"><col style="width:20%"><col style="width:10%"><col style="width:9%">' +
       '<col style="width:10%"><col style="width:9%"><col style="width:9%">' +
       '<col style="width:12%"><col style="width:16%"></colgroup>' +
       '<tr><th style="text-align:left">SF</th><th style="text-align:left">Pipeline</th>' +
       '<th>Frames</th><th>CRC%</th><th>SNR avg</th><th>min</th><th>max</th>' +
       '<th>CFO</th><th>|SFO|</th></tr>';

  const allSfs = sfData ? sfData.map(r => r.sf) : [];
  const pipeSfs = pipeData && pipeData.per_sf ? pipeData.per_sf.map(r => r.sf) : [];
  const sfs = [...new Set([...allSfs, ...pipeSfs])].sort((a,b) => a - b);

  sfs.forEach(sf => {
    const s = sfData ? sfData.find(r => r.sf === sf) : null;
    const pp = pipeMap[sf];
    // Funnel bar
    let bar = '';
    if (pp) {
      const nd = pp.detect||0, ns = pp.sync||0, nf = pp.frame||0;
      const wd = (nd/maxD*100).toFixed(1), ws = (ns/maxD*100).toFixed(1), wf = (nf/maxD*100).toFixed(1);
      bar = `<div class="funnel-bars">` +
            `<div class="fb fb-detect" style="width:${wd}%"></div>` +
            `<div class="fb fb-sync" style="width:${ws}%"></div>` +
            `<div class="fb fb-frame" style="width:${wf}%"></div></div>`;
    }
    h += `<tr><td>SF${sf}</td><td style="text-align:left">${bar}</td>` +
         `<td class="mono">${s ? s.frames : '-'}</td>` +
         `<td class="mono ${s ? crcClass(s.crc_pct) : ''}">${s ? fmt(s.crc_pct,1)+'%' : '-'}</td>` +
         `<td class="mono">${s ? fmt(s.avg_snr,1) : '-'}</td>` +
         `<td class="mono">${s ? fmt(s.min_snr,1) : '-'}</td>` +
         `<td class="mono">${s ? fmt(s.max_snr,1) : '-'}</td>` +
         `<td class="mono">${s ? fmt(s.avg_cfo_frac,3) : '-'}</td>` +
         `<td class="mono">${s ? fmt(s.avg_abs_sfo,5) : '-'}</td></tr>`;
  });
  h += '</table>';
  el.innerHTML = h;
}

function renderTrend(id, data, key, colorFn) {
  const el = document.getElementById(id);
  if (!data || !data.length) { el.innerHTML = ''; return; }
  const vals = data.map(r => r[key]);
  const valid = vals.filter(v => v !== null && v !== undefined);
  if (!valid.length) { el.innerHTML = ''; return; }
  const max = Math.max(...valid, 1);
  el.innerHTML = vals.map(v => {
    if (v === null || v === undefined) return '<div class="trend-bar" style="height:1px"></div>';
    const h = Math.max(1, (v / max) * 36);
    const cls = colorFn ? colorFn(v) : '';
    return `<div class="trend-bar ${cls}" style="height:${h}px" title="${v}"></div>`;
  }).join('');
}

function renderNoiseTrend(data) {
  const el = document.getElementById('trend-noise');
  if (!data || !data.length) { el.innerHTML = ''; return; }
  const vals = data.map(r => r.avg_noise_floor);
  const valid = vals.filter(v => v !== null && v !== undefined);
  if (!valid.length) { el.innerHTML = ''; return; }
  const absVals = valid.map(v => Math.abs(v));
  const mx = Math.max(...absVals, 1);
  el.innerHTML = vals.map(v => {
    if (v === null || v === undefined) return '<div class="trend-bar" style="height:1px"></div>';
    const h = Math.max(1, (Math.abs(v) / mx) * 36);
    return `<div class="trend-bar" style="height:${h}px;background:var(--accent);opacity:0.6" title="${v} dBFS"></div>`;
  }).join('');
}

// --- Sortable recent frames table ---
const RECENT_COLS = [
  {k:'ts',     l:'Time',     align:'left',  fmt: v => v ? v.substring(11,23) : '-'},
  {k:'seq',    l:'Seq',      fmt: v => v},
  {k:'sf',     l:'SF',       fmt: v => 'SF'+v},
  {k:'bw',     l:'BW',       fmt: v => (v/1000)+'k'},
  {k:'crc_valid', l:'CRC',   fmt: v => v ? '<span class="good">OK</span>' : '<span class="bad">FAIL</span>'},
  {k:'snr_db', l:'SNR',      fmt: v => fmt(v,1)},
  {k:'noise_floor_db', l:'Noise', fmt: v => fmt(v,1)},
  {k:'cfo_int', l:'CFO int', fmt: v => fmt(v,0)},
  {k:'cfo_frac', l:'CFO frac', fmt: v => fmt(v,3)},
  {k:'sfo_hat', l:'SFO',     fmt: v => fmt(v,5)},
  {k:'payload_len', l:'Len', fmt: v => v},
  {k:'decode_label', l:'Label', align:'left', fmt: v => v||'-'},
  {k:'device', l:'Device', align:'left', fmt: v => v ? v.substring(v.length-6) : '-'},
];

function sortRecent(colIdx) {
  if (sortCol === colIdx) { sortDir *= -1; }
  else { sortCol = colIdx; sortDir = 1; }
  renderRecentSorted();
}

function renderRecentSorted() {
  const el = document.getElementById('recent-table');
  if (!recentData || !recentData.length) { el.innerHTML = '<tr><td>no data</td></tr>'; return; }
  let rows = [...recentData];
  if (sortCol !== null) {
    const k = RECENT_COLS[sortCol].k;
    rows.sort((a,b) => {
      let va = a[k], vb = b[k];
      if (va === null || va === undefined) va = '';
      if (vb === null || vb === undefined) vb = '';
      if (typeof va === 'number' && typeof vb === 'number') return (va - vb) * sortDir;
      return String(va).localeCompare(String(vb)) * sortDir;
    });
  }
  let h = '<tr>' + RECENT_COLS.map((c,i) => {
    let cls = 'sortable';
    if (sortCol === i) cls += sortDir > 0 ? ' sort-asc' : ' sort-desc';
    const al = c.align === 'left' ? ' style="text-align:left"' : '';
    return `<th class="${cls}"${al} onclick="sortRecent(${i})">${c.l}</th>`;
  }).join('') + '</tr>';
  rows.forEach(r => {
    h += '<tr>' + RECENT_COLS.map(c => {
      const al = c.align === 'left' ? ' style="text-align:left"' : '';
      return `<td class="mono"${al}>${c.fmt(r[c.k])}</td>`;
    }).join('') + '</tr>';
  });
  el.innerHTML = h;
}

function renderRecent(data) {
  recentData = data || [];
  sortCol = null; sortDir = 1;
  renderRecentSorted();
}

async function refresh() {
  const t0 = Date.now();
  const [stats, frames, sf, pipeline, recent] = await Promise.all([
    q(`/api/stats?window=${W}`), q(`/api/frames?window=${W}`),
    q(`/api/sf?window=${W}`), q(`/api/pipeline?window=${W}`),
    q('/api/recent?limit=20'),
  ]);
  renderOverview(stats);
  renderSfPipeline(sf, pipeline);
  renderTrend('trend-frames', frames, 'frames');
  renderTrend('trend-crc', frames, 'crc_pct', v => v >= 90 ? 'good' : v >= 70 ? 'warn' : 'fail');
  renderTrend('trend-snr', frames, 'avg_snr');
  renderNoiseTrend(frames);
  renderRecent(recent);
  const ms = Date.now() - t0;
  document.getElementById('status').textContent =
    `window: ${W} | refreshed ${ms}ms | ${new Date().toLocaleTimeString('en-GB')}`;
}

refresh();
setInterval(refresh, 10000);
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run_collector(
    host: str, port: int, db_path: str, dashboard_port: int | None
) -> None:
    """Run the UDP-to-DuckDB collector with optional dashboard."""
    # Ensure parent directories exist
    db_dir = Path(db_path).parent
    db_dir.mkdir(parents=True, exist_ok=True)

    con = duckdb.connect(db_path)
    create_tables(con)
    log.info("DuckDB opened: %s", db_path)

    # Start dashboard if requested
    if dashboard_port is not None:
        start_dashboard(con, dashboard_port)

    # UDP setup -- subscribe without filter (all event types for telemetry)
    sock, sub_msg, addr = create_udp_subscriber(host, port)
    local_port = sock.getsockname()[1]
    log.info("subscribed to lora_trx at %s:%d (local :%d)", host, port, local_port)

    total = 0
    last_keepalive = time.monotonic()
    last_checkpoint = 0
    waiting = False

    try:
        while True:
            try:
                data, _addr = sock.recvfrom(65536)
                waiting = False
            except TimeoutError:
                sock.sendto(sub_msg, addr)
                last_keepalive = time.monotonic()
                if not waiting:
                    log.info("waiting for frames from %s:%d", host, port)
                    waiting = True
                continue

            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(sub_msg, addr)
                last_keepalive = now

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type", "")
            inserter = _INSERTERS.get(msg_type)  # type: ignore[arg-type]
            if inserter is None:
                continue

            total += 1
            inserter(con, msg)

            # Periodic DuckDB WAL checkpoint
            if total - last_checkpoint >= CHECKPOINT_INTERVAL:
                con.execute("CHECKPOINT")
                last_checkpoint = total

            # Log lora_frame events at debug level
            if msg_type == "lora_frame":
                phy = msg.get("phy", {})
                crc = "OK" if msg.get("crc_valid") else "FAIL"
                snr = phy.get("snr_db")
                snr_str = f" SNR={snr:.1f}" if snr is not None else ""
                log.debug(
                    "#%d seq=%s %dB CRC_%s%s",
                    total,
                    msg.get("seq", 0),
                    len(msg.get("payload", b"")),
                    crc,
                    snr_str,
                )

    except KeyboardInterrupt:
        pass
    finally:
        con.execute("CHECKPOINT")
        sock.close()
        con.close()

    log.info("%d events stored to %s", total, db_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa telemetry collector with DuckDB storage and live dashboard",
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=None,
        help="lora_trx UDP server address (default from config.toml or 127.0.0.1:5555)",
    )
    parser.add_argument(
        "--db",
        metavar="PATH",
        default=DEFAULT_DB_PATH,
        help=f"DuckDB database file (default: {DEFAULT_DB_PATH})",
    )
    parser.add_argument(
        "--dashboard-port",
        metavar="PORT",
        type=int,
        default=DEFAULT_DASHBOARD_PORT,
        help=f"Dashboard HTTP port (default: {DEFAULT_DASHBOARD_PORT})",
    )
    parser.add_argument(
        "--no-dashboard",
        action="store_true",
        help="Disable the web dashboard (collector only)",
    )
    add_logging_args(parser)
    args = parser.parse_args()

    setup_logging("gr4.duckdb", log_level=args.log_level, no_color=args.no_color)

    if args.connect:
        try:
            host, port = parse_host_port(args.connect)
        except ValueError as exc:
            parser.error(str(exc))
    else:
        host, port = "127.0.0.1", 5555

    dashboard_port = None if args.no_dashboard else args.dashboard_port
    run_collector(host, port, args.db, dashboard_port)


if __name__ == "__main__":
    main()
