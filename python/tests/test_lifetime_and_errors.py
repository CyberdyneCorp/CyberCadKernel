"""Handle lifetime (context managers, auto-release) and exception mapping.

The ABI signals failure by returning ``0`` / ``valid == 0``; the pythonic layer
turns those into :class:`CyberCadError`. Handles are released exactly once via
``close`` / ``__exit__`` / GC.
"""

import pytest

from cybercadkernel import CyberCadError, Kernel


def test_context_manager_releases_handle(box):
    with box(1, 1, 1) as b:
        assert b.id != 0
        assert not b.closed
    assert b.closed, "__exit__ must release the handle"


def test_close_is_idempotent(box):
    b = box(1, 1, 1)
    b.close()
    b.close()  # no crash, no double free
    assert b.closed


def test_id_after_close_raises(box):
    b = box(1, 1, 1)
    b.close()
    with pytest.raises(CyberCadError):
        _ = b.id  # accessing a released handle raises


def test_degenerate_extrude_raises(kernel: Kernel):
    # A collinear/degenerate profile cannot build a solid → the factory raises
    # rather than returning a 0 handle silently.
    with pytest.raises(CyberCadError):
        kernel.extrude([(0, 0), (0, 0), (0, 0)], 10.0)


def test_kernel_repr_and_last_error_are_strings(kernel: Kernel):
    assert isinstance(kernel.last_error, str)
    assert kernel.brep_available is True


def test_boolean_returns_new_shape_leaving_operands(box):
    a = box(4, 4, 4)
    b = box(4, 4, 4).translate(2, 2, 2)
    with a.fuse(b) as fused:
        assert fused.id not in (a.id, b.id)
        assert fused.mass_properties().valid
    # Operands are still usable after the boolean.
    assert a.mass_properties().volume == pytest.approx(64.0, abs=1e-6)
    assert b.mass_properties().volume == pytest.approx(64.0, abs=1e-6)
