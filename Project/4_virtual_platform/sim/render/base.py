"""Renderer protocol and factory.

A renderer is a *pure consumer*: it receives the world and reads ``S_t``.  It
must never mutate simulation state, which is what keeps visualisation, AI and
physics decoupled.  Renderers are optional dependencies - importing this module
never requires pygame.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from bullet_sim.core.errors import NotSupportedError


@runtime_checkable
class Renderer(Protocol):
    name: str

    def render(self, world: Any) -> Any: ...

    def close(self) -> None: ...


def make_renderer(mode: str = "human", **kwargs: Any) -> Renderer:
    """Build a renderer by mode name.

    ``human`` / ``rgb_array`` -> pygame (optional dependency)
    ``ascii``                 -> dependency-free text view
    ``none``                  -> :class:`NullRenderer`
    """
    if mode in (None, "none"):
        return NullRenderer()
    if mode == "ascii":
        from bullet_sim.render.ascii_view import AsciiRenderer

        return AsciiRenderer(**kwargs)
    if mode in ("human", "rgb_array"):
        try:
            from bullet_sim.render.pygame_view import PygameRenderer
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise NotSupportedError(
                "pygame is not installed; use mode='ascii', mode='none', or install pygame"
            ) from exc
        return PygameRenderer(mode=mode, **kwargs)
    raise NotSupportedError(f"unknown render mode {mode!r}")


class NullRenderer:
    """Renders nothing; useful as a default and for headless runs."""

    name = "none"

    def render(self, world: Any) -> None:
        return None

    def close(self) -> None:
        return None


__all__ = ["Renderer", "NullRenderer", "make_renderer"]
