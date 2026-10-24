"""规范一 · 角色。见 player.md / player.json / player.py。"""
from .player import (  # noqa: F401
    BHL1_PLAYER_FIELDS,
    DEFAULT_FRAME_ID,
    STANDARD_VERSION,
    PlayerState,
)

__all__ = ["PlayerState", "STANDARD_VERSION", "DEFAULT_FRAME_ID", "BHL1_PLAYER_FIELDS"]
