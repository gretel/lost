#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_duckdb.py -- LoRa frame collector with DuckDB storage.

Subscribes to lora_trx CBOR UDP frames and stores them in a local DuckDB
database. Captures PHY metrics (SNR, noise floor, CRC status) and raw
payload bytes for later analysis.

Usage:
    lora_duckdb.py                                    # defaults
    lora_duckdb.py --db data/lora_frames.duckdb       # custom db path

Dashboard (after stopping collector):
    duckdb data/lora_frames.duckdb -ui

Example queries:
    -- Frame rate per minute
    SELECT date_trunc('minute', ts) AS t, count(*)
    FROM lora_frames GROUP BY t ORDER BY t;

    -- CRC success rate per hour
    SELECT date_trunc('hour', ts) AS t,
           round(avg(crc_valid::int) * 100, 1) AS crc_pct
    FROM lora_frames GROUP BY t ORDER BY t;

    -- SNR distribution per hour
    SELECT date_trunc('hour', ts) AS t,
           round(avg(snr_db), 1), round(min(snr_db), 1), round(max(snr_db), 1)
    FROM lora_frames WHERE crc_valid GROUP BY t ORDER BY t;

    -- Noise floor trend (5-min buckets)
    SELECT time_bucket(INTERVAL '5 minutes', ts) AS t,
           round(avg(noise_floor_db), 1)
    FROM lora_frames GROUP BY t ORDER BY t;

    -- Recent frames
    SELECT ts, seq, sf, snr_db, crc_valid, payload_len
    FROM lora_frames ORDER BY ts DESC LIMIT 20;

Dependencies: cbor2, duckdb (both in project venv)
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path
from typing import Any

import cbor2
import duckdb

from lora_common import (
    KEEPALIVE_INTERVAL,
    create_udp_subscriber,
    load_config,
    resolve_udp_address,
    setup_logging,
)

DEFAULT_DB_PATH = "data/lora_frames.duckdb"

# Checkpoint every N inserts for WAL durability (DuckDB auto-checkpoints too,
# but explicit checkpoints keep the WAL small during long collection runs).
CHECKPOINT_INTERVAL = 100


# ---- Schema ----


def create_table(con: duckdb.DuckDBPyConnection) -> None:
    """Create the lora_frames table if it doesn't exist."""
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


# ---- Insert ----


_INSERT_SQL = """
    INSERT INTO lora_frames (
        seq, sf, bw, cr, sync_word, crc_valid, rx_channel,
        is_downchirp, snr_db, noise_floor_db, payload_len, payload_hex
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);
"""


def insert_frame(con: duckdb.DuckDBPyConnection, msg: dict[str, Any]) -> None:
    """Insert a lora_frame CBOR message into DuckDB."""
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    con.execute(
        _INSERT_SQL,
        [
            msg.get("seq", 0),
            phy.get("sf", 0),
            phy.get("bw", 0),
            phy.get("cr", 0),
            phy.get("sync_word", 0),
            msg.get("crc_valid", False),
            msg.get("rx_channel"),
            msg.get("is_downchirp", False),
            phy.get("snr_db"),
            phy.get("noise_floor_db"),
            len(payload),
            payload.hex() if payload else None,
        ],
    )


# ---- Main loop ----


log = logging.getLogger("gr4.duckdb")


def run_collector(host: str, port: int, db_path: str) -> None:
    """Run the UDP-to-DuckDB collector."""
    # Ensure parent directories exist
    db_dir = Path(db_path).parent
    db_dir.mkdir(parents=True, exist_ok=True)

    con = duckdb.connect(db_path)
    create_table(con)
    log.info("DuckDB opened: %s", db_path)

    # UDP setup -- subscribe without filter (all frames for telemetry)
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

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            total += 1
            insert_frame(con, msg)

            # Periodic DuckDB WAL checkpoint
            if total - last_checkpoint >= CHECKPOINT_INTERVAL:
                con.execute("CHECKPOINT")
                last_checkpoint = total

            # Status
            phy = msg.get("phy", {})
            crc = "OK" if msg.get("crc_valid") else "FAIL"
            snr = phy.get("snr_db")
            snr_str = f" SNR={snr:.1f}" if snr is not None else ""
            log.info(
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

    log.info("%d frames stored to %s", total, db_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa frame collector with DuckDB storage",
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
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output (also: NO_COLOR env var)",
    )
    args = parser.parse_args()

    cfg = load_config()
    setup_logging("gr4.duckdb", cfg, no_color=args.no_color)

    try:
        host, port = resolve_udp_address(args.connect, cfg)
    except ValueError as exc:
        parser.error(str(exc))

    run_collector(host, port, args.db)


if __name__ == "__main__":
    main()
