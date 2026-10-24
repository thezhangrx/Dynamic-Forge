"""CPU/FPGA state-action protocol: byte-exact encode/decode and validation."""

from __future__ import annotations

import struct
import zlib

import numpy as np
import pytest

from bullet_sim.core.errors import ProtocolError
from bullet_sim.entities.bullet import PROTOCOL_FIELDS_V1
from bullet_sim.interface.protocol import (
    BULLET_STRIDE,
    frame_size_v1,
    BULLET_STRIDE_V1,
    HEADER_BYTES,
    MAGIC,
    PROTOCOL_FIELDS,
    TRAILER_BYTES,
    _HEADER_STRUCT,
    fields_for,
    stride_for,
    bullet_matrix,
    decode_action_discrete,
    decode_action_velocity,
    decode_state,
    describe_protocol,
    encode_action_discrete,
    encode_action_velocity,
    encode_state,
    frame_size,
    read_header,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.world import World


def _world(level="medium", seed=2, steps=150):
    world = World(scenario_for_level(level, seed=seed), reward="zero",
                  terminate_on_collision=False)
    for _ in range(steps):
        world.step(0)
    return world


def test_frame_size_and_header_layout():
    world = _world()
    n = world.active_bullet_count()
    frame = encode_state(world.get_state())
    assert len(frame) == frame_size(n)
    assert len(frame) == 88 + BULLET_STRIDE * 4 * n
    # v2 added angle/angular_velocity/id; v3 added the DynamicObstacle geometry
    assert BULLET_STRIDE == 17
    header = read_header(frame)
    assert header.magic == MAGIC
    assert header.version == 3
    assert header.n_bullets == n
    assert header.step_index == world.state.env.step_index
    assert header.field_w == pytest.approx(world.spec.field_w)
    assert HEADER_BYTES == 84 and TRAILER_BYTES == 4


def test_state_frame_roundtrip_preserves_all_fields():
    world = _world()
    snap = world.get_state()
    frame = encode_state(snap)
    decoded = decode_state(frame)

    assert decoded.bullet_count == snap.bullet_count
    assert decoded.env.step_index == snap.env.step_index
    assert decoded.env.sim_time == pytest.approx(snap.env.sim_time, abs=1e-5)
    assert decoded.player.to_array() == pytest.approx(snap.player.to_array(), abs=1e-5)
    assert decoded.target.to_array() == pytest.approx(snap.target.to_array(), abs=1e-5)

    a = snap.bullets.to_protocol_matrix()
    b = decoded.bullets.to_protocol_matrix()
    assert a.shape == b.shape == (snap.bullet_count, len(PROTOCOL_FIELDS))
    # the wire format is float32, so compare at float32 precision
    assert np.allclose(a, b, atol=1e-4)


def test_bullet_matrix_view_matches_payload():
    world = _world(level="easy", steps=100)
    frame = encode_state(world.get_state())
    view = bullet_matrix(frame)
    assert view.shape == (world.active_bullet_count(), len(PROTOCOL_FIELDS))
    decoded = decode_state(frame)
    assert np.allclose(view, decoded.bullets.to_protocol_matrix(), atol=1e-6)


def test_crc_corruption_is_detected():
    world = _world(level="easy", steps=60)
    frame = bytearray(encode_state(world.get_state()))
    # flip a byte inside the bullet payload
    frame[HEADER_BYTES + 1] ^= 0xFF
    with pytest.raises(ProtocolError, match="CRC"):
        decode_state(bytes(frame))


def test_bad_magic_and_version_are_rejected():
    world = _world(level="easy", steps=20)
    frame = bytearray(encode_state(world.get_state()))
    struct.pack_into("<I", frame, 0, 0xDEADBEEF)
    with pytest.raises(ProtocolError, match="magic"):
        decode_state(bytes(frame))

    frame = bytearray(encode_state(world.get_state()))
    struct.pack_into("<H", frame, 4, 99)
    with pytest.raises(ProtocolError, match="version"):
        decode_state(bytes(frame))


def test_truncated_and_oversized_frames_are_rejected():
    world = _world(level="easy", steps=20)
    frame = encode_state(world.get_state())
    with pytest.raises(ProtocolError):
        decode_state(frame[:-1])
    with pytest.raises(ProtocolError):
        decode_state(frame + b"\x00")


def test_empty_state_frame():
    world = World(scenario_for_level("easy", seed=1), reward="zero")
    world.state.bullets.clear()
    frame = encode_state(world.get_state())
    assert len(frame) == 88
    decoded = decode_state(frame)
    assert decoded.bullet_count == 0


def test_action_frames():
    v = encode_action_velocity(12.5, -7.25)
    assert len(v) == 8
    assert np.allclose(decode_action_velocity(v), [12.5, -7.25])
    d = encode_action_discrete(7)
    assert len(d) == 4 and decode_action_discrete(d) == 7
    with pytest.raises(ProtocolError):
        decode_action_velocity(b"\x00")
    with pytest.raises(ProtocolError):
        decode_action_discrete(b"\x00")


def test_protocol_description_is_complete():
    info = describe_protocol()
    assert info["version"] == 3
    assert info["readable_versions"] == [1, 2, 3]
    assert info["header_bytes"] == 84
    assert info["bullet_stride_floats"] == 17
    assert info["bullet_stride_floats"] == BULLET_STRIDE
    assert tuple(info["bullet_fields"]) == PROTOCOL_FIELDS
    assert tuple(info["bullet_fields_v1"]) == PROTOCOL_FIELDS_V1
    assert "angle" in PROTOCOL_FIELDS and "angular_velocity" in PROTOCOL_FIELDS
    assert "id" in PROTOCOL_FIELDS
    # v3: a bullet is a DynamicObstacle and can be a rotated rectangle
    for field in ("shape", "half_w", "half_h", "rotation"):
        assert field in PROTOCOL_FIELDS


def test_v1_frames_are_still_readable():
    """An old capture (10-float stride) must decode into a valid snapshot."""
    world = _world(level="easy", steps=60)
    snap = world.get_state()
    n = snap.bullet_count
    v1 = bytearray(frame_size_v1(n))
    _HEADER_STRUCT.pack_into(
        v1, 0, MAGIC, 1, 0, int(snap.env.step_index), n,
        float(snap.env.sim_time), snap.env.field_w, snap.env.field_h, snap.env.dt,
    )
    off = _HEADER_STRUCT.size
    off += 4 * 7
    off += 4 * 6
    mat = snap.bullets.to_protocol_matrix()[:, :BULLET_STRIDE_V1]
    v1[off : off + mat.nbytes] = np.asarray(mat, dtype="<f4").tobytes()
    off += mat.nbytes
    struct.pack_into("<I", v1, off, zlib.crc32(bytes(v1[:off])) & 0xFFFFFFFF)

    decoded = decode_state(bytes(v1))
    assert decoded.bullet_count == n
    # the missing v2 fields are derived, not invented
    assert np.all(decoded.bullets.data["angular_velocity"][:n] == 0.0)
    assert np.array_equal(decoded.bullets.data["id"][:n], np.arange(1, n + 1))
    compact = snap.bullets.to_protocol_matrix()  # canonical live ordering
    assert np.allclose(
        decoded.bullets.data["angle"][:n],
        np.arctan2(compact[:, 3], compact[:, 2]),
        atol=1e-6,
    )


def test_stride_and_field_registry():
    assert stride_for(1) == 10 and stride_for(2) == 13
    assert len(fields_for(1)) == 10 and len(fields_for(2)) == 13
    with pytest.raises(Exception):
        stride_for(99)


def test_roundtrip_preserves_bullet_ids_and_angle():
    world = _world(level="easy", steps=80)
    snap = world.get_state()
    decoded = decode_state(encode_state(snap))
    n = snap.bullet_count
    assert np.array_equal(
        decoded.bullets.data["id"][:n], snap.bullets.data["id"][:n]
    )
    assert np.allclose(
        decoded.bullets.data["angle"][:n], snap.bullets.data["angle"][:n], atol=1e-5
    )


def test_hardware_link_loopback_and_stats(tmp_path):
    from bullet_sim.interface.hardware import FileLink, LoopbackLink, make_link

    world = _world(level="easy", steps=40)
    frame = encode_state(world.get_state())

    link = LoopbackLink(accelerator=lambda f: f)
    link.open()
    out = link.roundtrip(frame)
    assert out == frame
    link.send_action(np.array([1.0, 0.0]))
    stats = link.stats.summary()
    assert stats["uploads"] == 1 and stats["downloads"] == 1 and stats["actions"] == 1
    assert stats["bytes_up"] == len(frame)

    file_link = FileLink(tmp_path / "tx.bin", tmp_path / "rx.bin")
    file_link.upload_state(frame)
    (tmp_path / "rx.bin").write_bytes(struct.pack("<I", len(frame)) + frame)
    assert file_link.fetch_prediction() == frame

    assert make_link("null").name == "null"
    with pytest.raises(ValueError):
        make_link("carrier-pigeon")
