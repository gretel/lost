#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
serial_bridge.py -- DTR-safe serial-to-TCP bridge for MeshCore companion.

Opens the serial port ONCE with DTR/RTS suppressed (preventing ESP32 reboot),
then bridges bidirectionally to a TCP server. meshcore-cli connects via TCP
instead of serial, avoiding the DTR reboot on every invocation.

Usage:
    python3 scripts/apps/serial_bridge.py [--serial PORT] [--baud RATE] [--tcp-port PORT]

Then:
    uvx meshcore-cli -t 127.0.0.1 -p 7835 advert
"""

from __future__ import annotations

import argparse
import asyncio
import sys

import serial


async def main() -> None:
    parser = argparse.ArgumentParser(description="DTR-safe serial-to-TCP bridge")
    parser.add_argument(
        "--serial", default="/dev/cu.usbserial-0001", help="serial port"
    )
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--tcp-port", type=int, default=7835, help="TCP listen port")
    args = parser.parse_args()

    # Open serial with DTR/RTS suppressed BEFORE open() to prevent ESP32 reboot
    ser = serial.Serial()
    ser.port = args.serial
    ser.baudrate = args.baud
    ser.timeout = 0.01
    ser.dtr = False
    ser.rts = False
    ser.open()
    print(f"serial: {args.serial} @ {args.baud} (DTR/RTS suppressed)", flush=True)

    loop = asyncio.get_event_loop()

    async def handle_client(
        reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        print(f"client connected: {peer}", flush=True)

        async def serial_to_tcp() -> None:
            while not reader.at_eof():
                data = await loop.run_in_executor(
                    None, lambda: ser.read(ser.in_waiting or 1)
                )
                if data:
                    writer.write(data)
                    await writer.drain()
                else:
                    await asyncio.sleep(0.001)

        async def tcp_to_serial() -> None:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                ser.write(data)

        t1 = asyncio.create_task(serial_to_tcp())
        t2 = asyncio.create_task(tcp_to_serial())
        done, pending = await asyncio.wait(
            [t1, t2], return_when=asyncio.FIRST_COMPLETED
        )
        for t in pending:
            t.cancel()
        writer.close()
        print(f"client disconnected: {peer}", flush=True)

    server = await asyncio.start_server(handle_client, "127.0.0.1", args.tcp_port)
    print(f"bridge: TCP:{args.tcp_port} <-> {args.serial}", flush=True)
    try:
        await server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    asyncio.run(main())
