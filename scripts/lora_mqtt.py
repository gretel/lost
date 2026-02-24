#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_mqtt.py -- LoRa-to-MQTT bridge for letsmesh.net.

Subscribes to lora_trx CBOR UDP frames and publishes them to an MQTT
broker in the meshcoretomqtt JSON format used by letsmesh.net packet
analyzers. TLS is always enabled (port 443 by default).

Duplicate frames from dual-channel RX are suppressed.

Usage:
    lora_mqtt.py --iata BER
    lora_mqtt.py --iata BER --connect 127.0.0.1:5555
    lora_mqtt.py --iata BER --mqtt-host mqtt-eu-v1.letsmesh.net

meshcoretomqtt JSON format (published per frame):
    {
        "raw": "<hex>",
        "len": <int>,
        "type": <int>,
        "route": <int>,
        "payload_len": <int>,
        "snr": <float>,
        "rssi": <float>,
        "timestamp": "<ISO 8601>"
    }

MQTT topic: meshcore/<IATA>/<PUBLIC_KEY>/packets

Dependencies: cbor2, paho-mqtt, pynacl, pycryptodome
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any

import cbor2
import paho.mqtt.client as mqtt

from meshcore_crypto import (
    DEFAULT_IDENTITY_FILE,
    load_or_create_identity,
)

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555
DEFAULT_MQTT_HOST = "mqtt-eu-v1.letsmesh.net"
DEFAULT_MQTT_PORT = 443
KEEPALIVE_INTERVAL = 5.0
RECV_TIMEOUT = 10.0
DEDUP_WINDOW = 2.0  # seconds; suppress duplicate payloads within this window


def frame_to_meshcore_json(msg: dict[str, Any]) -> dict[str, Any] | None:
    """Convert a lora_frame CBOR message to meshcoretomqtt JSON format."""
    payload = msg.get("payload", b"")
    if not payload or not msg.get("crc_valid", False):
        return None

    phy = msg.get("phy", {})

    # Parse MeshCore header
    if len(payload) < 2:
        return None
    hdr = payload[0]
    route = hdr & 0x03
    ptype = (hdr >> 2) & 0x0F

    # Determine payload_len (after header + path_len prefix)
    has_transport = route in (0, 3)
    off = 1
    if has_transport:
        off += 4
    if off < len(payload):
        path_len = payload[off]
        off += 1 + path_len
    payload_len = len(payload) - off

    return {
        "raw": payload.hex().upper(),
        "len": len(payload),
        "type": ptype,
        "route": route,
        "payload_len": payload_len,
        "snr": phy.get("snr_db", 0.0),
        "rssi": -120.0 + phy.get("snr_db", 0.0),  # approximate; no RSSI from SDR
        "timestamp": msg.get("ts", ""),
    }


def run_bridge(
    host: str,
    port: int,
    mqtt_host: str,
    mqtt_port: int,
    iata: str,
    pub_key_hex: str,
    topic_prefix: str,
) -> None:
    """Run the UDP-to-MQTT bridge."""
    # MQTT setup (TLS always)
    topic = f"{topic_prefix}/{iata}/{pub_key_hex}/packets"
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.tls_set()

    sys.stderr.write(f"Connecting to MQTT broker {mqtt_host}:{mqtt_port} (TLS)...\n")
    try:
        client.connect(mqtt_host, mqtt_port, keepalive=60)
    except Exception as exc:
        sys.stderr.write(f"MQTT connect failed: {exc}\n")
        sys.exit(1)

    client.loop_start()
    sys.stderr.write(f"MQTT connected, topic: {topic}\n")

    # UDP setup
    af = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    sock.settimeout(RECV_TIMEOUT)
    local_port = sock.getsockname()[1]

    sock.sendto(b"sub", (host, port))
    sys.stderr.write(
        f"Subscribed to lora_trx at {host}:{port} (local port :{local_port})\n"
    )

    published = 0
    deduplicated = 0
    last_keepalive = time.monotonic()
    # Dedup: (payload_hex, timestamp) of recently published frames
    recent: dict[str, float] = {}

    try:
        while True:
            try:
                data, _addr = sock.recvfrom(65536)
            except socket.timeout:
                sock.sendto(b"sub", (host, port))
                last_keepalive = time.monotonic()
                continue

            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(b"sub", (host, port))
                last_keepalive = now

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            mqtt_msg = frame_to_meshcore_json(msg)
            if mqtt_msg is None:
                continue

            # Dedup: skip if we published the same payload recently
            raw_hex = mqtt_msg["raw"]
            if raw_hex in recent and now - recent[raw_hex] < DEDUP_WINDOW:
                deduplicated += 1
                continue

            # Expire old entries
            recent = {k: v for k, v in recent.items() if now - v < DEDUP_WINDOW}
            recent[raw_hex] = now

            payload_json = json.dumps(mqtt_msg)
            client.publish(topic, payload_json)
            published += 1

            sys.stderr.write(
                f"  Published #{published}: {mqtt_msg['len']}B "
                f"type={mqtt_msg['type']} route={mqtt_msg['route']} "
                f"SNR={mqtt_msg['snr']:.1f}dB\n"
            )

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        client.loop_stop()
        client.disconnect()

    sys.stderr.write(
        f"\n--- {published} frames published, {deduplicated} duplicates suppressed\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa-to-MQTT bridge for letsmesh.net packet analyzer",
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=f"{DEFAULT_HOST}:{DEFAULT_PORT}",
        help=f"lora_trx UDP server address (default: {DEFAULT_HOST}:{DEFAULT_PORT})",
    )
    parser.add_argument(
        "--mqtt-host",
        default=DEFAULT_MQTT_HOST,
        help=f"MQTT broker hostname (default: {DEFAULT_MQTT_HOST})",
    )
    parser.add_argument(
        "--mqtt-port",
        type=int,
        default=DEFAULT_MQTT_PORT,
        help=f"MQTT broker port (default: {DEFAULT_MQTT_PORT})",
    )
    parser.add_argument(
        "--iata",
        required=True,
        help="IATA airport code for this station (e.g. BER, FRA, LHR)",
    )
    parser.add_argument(
        "--identity",
        type=Path,
        default=DEFAULT_IDENTITY_FILE,
        help=f"Identity file for public key (default: {DEFAULT_IDENTITY_FILE})",
    )
    parser.add_argument(
        "--topic-prefix",
        default="meshcore",
        help="MQTT topic prefix (default: meshcore)",
    )
    args = parser.parse_args()

    # Parse host:port
    spec = args.connect
    if spec.startswith("["):
        bracket = spec.index("]")
        host = spec[1:bracket]
        port = int(spec[bracket + 2 :])
    else:
        colon = spec.rfind(":")
        if colon <= 0:
            parser.error("--connect requires HOST:PORT (e.g. 127.0.0.1:5555)")
        host = spec[:colon]
        port = int(spec[colon + 1 :])

    # Load identity for public key
    expanded_prv, pub_key, seed = load_or_create_identity(args.identity)
    pub_key_hex = pub_key.hex()

    run_bridge(
        host,
        port,
        args.mqtt_host,
        args.mqtt_port,
        args.iata,
        pub_key_hex,
        args.topic_prefix,
    )


if __name__ == "__main__":
    main()
