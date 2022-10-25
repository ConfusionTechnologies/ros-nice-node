"""Miscellaneous utilities."""

import array
from typing import TYPE_CHECKING, Set

from ..types import DependenciesType

# numpy is an optional dependency
if TYPE_CHECKING:
    import numpy as np

__all__ = ["Symbol", "append_array", "should_update"]


class Symbol:
    """https://stackoverflow.com/questions/60696424/python-equivalent-to-js-symbol"""

    def __init__(self, name=""):
        self.name = f"<symbol {name}>"

    def __repr__(self):
        return self.name


def append_array(arr: array.array, np_arr: "np.ndarray", dtype: type = ...):
    """Append to builtin array from numpy array, avoiding copy where possible.

    Args:
        arr (array.array): Python array to append to.
        np_arr (np.ndarray): Numpy array to append from.
        dtype (Type, optional): Dtype of Numpy array. Will preserve original dtype if `...`. Defaults to `...`.

    Returns:
        _type_: _description_
    """
    arr.frombytes(
        np_arr.astype(
            np_arr.dtype if dtype is ... else dtype, order="C", copy=False
        ).data.cast("b")
    )
    return arr


def should_update(deps: DependenciesType, changes: Set[str]):
    """Given dependencies and a list of changes, determine if callback should trigger."""
    return deps is not None and (deps is ... or bool(changes & deps))
