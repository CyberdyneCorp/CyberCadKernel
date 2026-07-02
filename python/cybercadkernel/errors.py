"""Exceptions raised by the pythonic API.

The C ABI signals failure by returning ``0`` (an invalid ``CCShapeId``), an
empty struct (``valid == 0``), or ``0`` from a predicate, and stashes a
human-readable message on the current thread reachable via ``cc_last_error()``.
The pythonic layer turns those sentinels into exceptions carrying that message.
"""

from __future__ import annotations


class KernelError(RuntimeError):
    """A ``cc_*`` call failed. ``message`` is ``cc_last_error()`` (may be empty)."""

    def __init__(self, op: str, message: str = "") -> None:
        self.op = op
        self.message = message
        detail = f": {message}" if message else ""
        super().__init__(f"{op} failed{detail}")


class BRepUnavailableError(KernelError):
    """A B-rep operation was attempted on a build with no geometry engine
    (``cc_brep_available() == 0`` — the host stub)."""
