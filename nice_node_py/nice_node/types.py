from typing import *

__all__ = ["CfgType", "CallbackType", "EllipsisType", "CallbackListType"]
CfgType = TypeVar("CfgType")
"""Generic for config struct."""
CallbackType = Callable[[Dict[str, Any]], Any]
EllipsisType = type(...)
CallbackListType = List[Tuple[CallbackType, Union[None, EllipsisType, List[str]]]]
