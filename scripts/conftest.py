#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
conftest.py -- Shared test fixtures for meshcore test suite.

Provides reusable helpers for identity creation, BridgeState construction,
mock sockets, deterministic test seeds, and packet builders.  Individual
test files still carry their own local copies for now; this module is the
canonical location and will replace the duplicates in a follow-up.

Usage from any test_*.py in the same directory:

    import conftest
    state = conftest.make_state(sf=12)
    alice, bob = conftest.make_two_identities()
"""

from __future__ import annotations

import os
import socket
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

import meshcore_bridge as bridge
from meshcore_crypto import (
    load_or_create_identity,
    meshcore_expanded_key,
    meshcore_shared_secret,
    meshcore_encrypt_then_mac,
    GroupChannel,
    PAYLOAD_TXT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_GRP_TXT,
    PAYLOAD_ACK,
    PAYLOAD_ADVERT,
    PATH_HASH_SIZE,
)
from meshcore_tx import (
    build_txt_msg,
    build_anon_req,
    build_advert,
    build_wire_packet,
    make_header,
)


# ---------------------------------------------------------------------------
# Identity helpers
# ---------------------------------------------------------------------------


def make_identity():
    """Create a temporary identity for testing."""
    tmpdir = tempfile.mkdtemp()
    path = Path(tmpdir) / "identity.bin"
    expanded_prv, pub_key, seed = load_or_create_identity(path)
    return expanded_prv, pub_key, seed


def make_two_identities():
    """Create two identities (sender + receiver) for crypto tests."""
    d1 = tempfile.mkdtemp()
    d2 = tempfile.mkdtemp()
    prv_a, pub_a, seed_a = load_or_create_identity(Path(d1) / "a.bin")
    prv_b, pub_b, seed_b = load_or_create_identity(Path(d2) / "b.bin")
    return (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b)


# ---------------------------------------------------------------------------
# BridgeState factory
# ---------------------------------------------------------------------------


def make_state(**kwargs):
    """Create a BridgeState with test defaults using temp directories."""
    expanded_prv, pub_key, seed = make_identity()
    tmpdir = Path(tempfile.mkdtemp())
    defaults = dict(
        pub_key=pub_key,
        expanded_prv=expanded_prv,
        seed=seed,
        name="test-node",
        freq_mhz=869.618,
        bw_khz=62.5,
        sf=8,
        cr=4,
        tx_power=14,
        keys_dir=tmpdir / "keys",
        channels_dir=tmpdir / "channels",
        contacts_dir=tmpdir / "contacts",
    )
    defaults.update(kwargs)
    return bridge.BridgeState(**defaults)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Mock socket
# ---------------------------------------------------------------------------


class FakeUDPSock(socket.socket):
    """Minimal UDP socket mock that captures sendto calls."""

    def __init__(self) -> None:
        super().__init__(socket.AF_INET, socket.SOCK_DGRAM)
        self.close()  # release the FD immediately; we only need the type
        self.sent: list[bytes] = []

    def sendto(self, data: bytes, addr: object) -> int:  # type: ignore[override]
        self.sent.append(data)
        return len(data)


# ---------------------------------------------------------------------------
# Deterministic test seeds
# ---------------------------------------------------------------------------

TEST_SEED = bytes.fromhex(
    "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
)
TEST_SEED_PUB = bytes.fromhex(
    "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
)
TEST_SEED_EXPANDED = meshcore_expanded_key(TEST_SEED)


# ---------------------------------------------------------------------------
# Packet builder helpers
# ---------------------------------------------------------------------------


def build_test_advert_packet(seed: bytes, pub: bytes, name: str = "") -> bytes:
    """Build a minimal ADVERT packet for testing extraction functions."""
    return build_advert(seed, pub, name=name)


def build_test_txt_msg_packet(
    sender_expanded: bytes,
    sender_pub: bytes,
    dest_pub: bytes,
    text: str,
    timestamp: int = 1700000000,
) -> bytes:
    """Build a TXT_MSG packet for testing decryption."""
    return build_txt_msg(
        sender_expanded, sender_pub, dest_pub, text, timestamp=timestamp
    )


def build_test_anon_req_packet(
    sender_expanded: bytes,
    sender_pub: bytes,
    dest_pub: bytes,
    message: bytes,
    timestamp: int = 1700000000,
) -> bytes:
    """Build an ANON_REQ packet for testing decryption."""
    return build_anon_req(
        sender_expanded, sender_pub, dest_pub, message, timestamp=timestamp
    )
