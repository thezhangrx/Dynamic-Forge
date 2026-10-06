"""规范二 · 障碍。见 obstacle.md / obstacle.json / obstacle.py。"""
from .obstacle import (  # noqa: F401
    BHL1_BULLET_FIELDS,
    SHAPE_CODES,
    STANDARD_VERSION,
    TYPE_CODES,
    TYPE_NAMES,
    Obstacle,
    ObstacleShape,
    ObstacleType,
)

__all__ = ["Obstacle", "ObstacleShape", "ObstacleType", "SHAPE_CODES",
           "TYPE_CODES", "TYPE_NAMES", "BHL1_BULLET_FIELDS", "STANDARD_VERSION"]
