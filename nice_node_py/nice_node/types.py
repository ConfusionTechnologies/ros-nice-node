from types import EllipsisType
from typing import *

__all__ = ["CfgType", "CallbackType", "EllipsisType", "CallbackListType"]
CfgType = TypeVar("CfgType")
"""Generic for config struct."""
CallbackType = Callable[[Dict[str, Any]], Any]
"""Type for NiceNode callback which accepts a map of changed parameter names to values."""
DependenciesType = Union[None, EllipsisType, Set[str]]
"""Internal type for dependencies list."""
CallbackListType = List[Tuple[CallbackType, DependenciesType]]
"""Internal type for list of NiceNode callbacks."""
