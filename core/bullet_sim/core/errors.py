"""Exception hierarchy for the simulator platform."""


class BulletSimError(Exception):
    """Base class for all simulator errors."""


class ConfigError(BulletSimError):
    """Invalid or inconsistent configuration / scenario specification."""


class ScenarioError(BulletSimError):
    """Scenario cannot be built (bad pattern, unsatisfiable schedule, ...)."""


class StateError(BulletSimError):
    """State snapshot is malformed or incompatible."""


class ProtocolError(BulletSimError):
    """Flat State/Action buffer failed validation (magic/version/CRC/size)."""


class DatasetError(BulletSimError):
    """Dataset write/read failure or schema mismatch."""


class NotSupportedError(BulletSimError):
    """Requested optional capability is unavailable (e.g. pygame missing)."""
