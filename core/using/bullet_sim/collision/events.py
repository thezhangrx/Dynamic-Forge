"""Contact / collision **events**, separated from collision *detection*.

Detection answers "does the player overlap a bullet on this frame?".
This module answers the question the reward and the metrics actually need:
**"did a new collision event start?"**

Why it matters
--------------
A player that overlaps one bullet for 40 consecutive frames is *one* collision,
not 40.  Billing the penalty, or incrementing the counter, on every overlapping
frame makes the reward a function of the frame rate instead of the behaviour,
and makes ``collision_count`` meaningless.

The state machine is therefore explicit and lives in the collision module::

    no contact          --enter-->  contact        (collision_count += 1)
    contact, same ids   --hold-->   contact        (nothing counted)
    contact             --exit-->   no contact
    new bullet touches  --enter-->  contact        (collision_count += 1)

Contact is tracked **per bullet id**, so touching a second bullet while the
first is still touching is a second event, and separating from one bullet while
another still touches does not create a spurious "re-entry" later.

Counting modes
--------------
``per_contact`` (default)
    one event per newly touched bullet - the informative choice for learning.
``per_step``
    one event per contact *episode* of the player, regardless of how many
    bullets are involved.
``per_frame``
    legacy behaviour: every overlapping frame counts.  Kept so that older
    "frames survived in contact" experiments remain expressible without
    touching the kernel.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence

#: Accepted values for ``collision_count_mode``.
COUNT_MODES: tuple[str, ...] = ("per_contact", "per_step", "per_frame")


def validate_count_mode(mode: str) -> str:
    key = str(mode).strip().lower()
    if key not in COUNT_MODES:
        raise ValueError(
            f"unknown collision_count_mode {mode!r}; expected one of {list(COUNT_MODES)}"
        )
    return key


@dataclass(frozen=True)
class ContactTransition:
    """What changed between the previous step and this one."""

    frame: int
    #: Bullet ids that started touching on this frame.
    entered: tuple[int, ...] = ()
    #: Bullet ids that stopped touching on this frame.
    exited: tuple[int, ...] = ()
    #: Every bullet id currently touching.
    active: tuple[int, ...] = ()
    #: New collision events to bill / count on this step (mode dependent).
    events: int = 0
    #: Whether the player overlaps anything at all right now.
    hit: bool = False

    @property
    def is_new_event(self) -> bool:
        return self.events > 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "entered": list(self.entered),
            "exited": list(self.exited),
            "active": list(self.active),
            "events": self.events,
            "hit": self.hit,
        }


class ContactTracker:
    """Per-bullet enter/exit state machine for the player's hitbox."""

    def __init__(self, mode: str = "per_contact") -> None:
        self.mode = validate_count_mode(mode)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._active: set[int] = set()
        self.collision_count = 0
        self.overlap_frames = 0
        self.contact_frames = 0
        self.first_event_frame: int | None = None
        self.last_event_frame: int | None = None
        self.entered_total = 0
        self.exited_total = 0
        self.last_transition = ContactTransition(frame=-1)

    # ------------------------------------------------------------------
    def update(self, hit_ids: Iterable[int] | None, frame: int) -> ContactTransition:
        """Advance the state machine with the ids overlapping on ``frame``."""
        current = {int(i) for i in (hit_ids or ())}
        entered = current - self._active
        exited = self._active - current
        self._active = current

        if self.mode == "per_contact":
            events = len(entered)
        elif self.mode == "per_step":
            events = 1 if (entered and self.contact_frames == 0) else 0
        else:  # per_frame
            events = 1 if current else 0

        self.collision_count += events
        self.entered_total += len(entered)
        self.exited_total += len(exited)
        if current:
            self.overlap_frames += 1
            self.contact_frames += 1
        else:
            self.contact_frames = 0
        if events:
            self.first_event_frame = (
                frame if self.first_event_frame is None else self.first_event_frame
            )
            self.last_event_frame = frame

        self.last_transition = ContactTransition(
            frame=int(frame),
            entered=tuple(sorted(entered)),
            exited=tuple(sorted(exited)),
            active=tuple(sorted(current)),
            events=int(events),
            hit=bool(current),
        )
        return self.last_transition

    # ------------------------------------------------------------------
    @property
    def active_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self._active))

    @property
    def in_contact(self) -> bool:
        return bool(self._active)

    @property
    def active_count(self) -> int:
        return len(self._active)

    def frames_since_event(self, frame: int) -> int | None:
        if self.last_event_frame is None:
            return None
        return int(frame) - int(self.last_event_frame)

    def summary(self, frame: int | None = None, dt: float = 0.0) -> dict[str, Any]:
        out: dict[str, Any] = {
            "collision_count": int(self.collision_count),
            "collision_count_mode": self.mode,
            "active_ids": list(self.active_ids),
            "in_contact": self.in_contact,
            "overlap_frames": int(self.overlap_frames),
            "entered_total": int(self.entered_total),
            "exited_total": int(self.exited_total),
            "first_event_frame": self.first_event_frame,
            "last_event_frame": self.last_event_frame,
        }
        if frame is not None:
            out["frames_since_event"] = self.frames_since_event(frame)
            out["seconds_since_event"] = (
                None
                if out["frames_since_event"] is None
                else out["frames_since_event"] * float(dt)
            )
        return out


def contact_summary(tracker: ContactTracker, *, frame: int, dt: float) -> dict[str, Any]:
    """Convenience wrapper used by ``World`` for its ``info`` payload."""
    return tracker.summary(frame=frame, dt=dt)


__all__ = [
    "COUNT_MODES",
    "ContactTransition",
    "ContactTracker",
    "contact_summary",
    "validate_count_mode",
]
