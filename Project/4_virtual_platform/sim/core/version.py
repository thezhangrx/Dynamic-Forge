"""Single source of truth for package version and on-the-wire protocol version."""

__version__ = "0.1.0"

#: Version of the flat State/Action byte protocol (see interface/protocol.py).
#: v2 added the bullet fields angle / angular_velocity / id (stride 10 -> 13).
#: v3 turns a bullet into a DynamicObstacle: shape / half_w / half_h / rotation
#: (stride 13 -> 17).  ``decode_state`` still accepts v1 and v2 frames.
STATE_PROTOCOL_VERSION = 3
STATE_PROTOCOL_VERSION_MIN_READABLE = 1

#: Version of the exported dataset schema (see dataset/schema.py).
DATASET_SCHEMA_VERSION = 1
