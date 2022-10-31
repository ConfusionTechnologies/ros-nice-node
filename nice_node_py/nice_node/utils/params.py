"""Utilities for manipulating ROS2 node parameters."""

import json
from collections import deque
from copy import deepcopy
from dataclasses import is_dataclass
from typing import Any, Mapping, Sequence, Set, Union

import attr

# TODO: support user passing in their own encoder/decoder beyond json

__all__ = ["params_from_struct", "struct_from_params"]


def should_encode(v, types):
    """Whether a value should be transparently encoded to a str.

    Args:
        v (Any): Value to check.
        types (list): List of types to encode.

    Returns:
        bool: Whether to encode the value.
    """

    return not isinstance(v, str) and any(isinstance(v, t) for t in types)


def params_from_struct(
    struct: Union[object, type],
    *,
    exclude_keys: Set[str] = set(),
    encode_types=[Sequence, Mapping],
    namespace="",
):
    """Determines parameters to be declared based on a config struct (e.g. dataclasses, attrs).

    Recursion is done when the parameter value is itself a struct. Certain types of
    values will also be transparently converted to json to allow for easier editing.
    By default, these are `Sequence` (i.e. `list`, `tuple`) and `Mapping` (i.e. `dict`).

    Args:
        struct (Union[object, type]): Struct or instance of one.
        exclude_keys (Set[str], optional): Keys to exclude using dot notation (i.e. `foo.bar`). Defaults to [].
        encode_types (list, optional): List of types to encode. Defaults to [Sequence, Mapping].
        namespace (str, optional): Namespace for parameters. Defaults to "".

    Returns:
        List[Tuple[str, Any]]: List of parameter name to value to use for declaration.
    """

    # instantiate config struct if class was given
    struct = struct() if isinstance(struct, type) else struct
    exclude_keys = set(exclude_keys)  # for user convenience

    # exclude parameters that are declared by default beforehand
    exclude_keys.add("use_sim_time")

    # params to declare
    params = []
    for k, v in struct.__dict__.items():
        # ROS2 uses dot notation for parameter namespacing
        # See https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.declare_parameters
        ns = ".".join([namespace, k]) if namespace != "" else k
        if ns in exclude_keys:
            continue

        # recursively declare parameters if is struct
        if is_dataclass(v) or attr.has(type(v)):
            params += params_from_struct(
                v,
                exclude_keys=exclude_keys,
                encode_types=encode_types,
                namespace=ns,
            )
            continue
        # convert list & dicts to str for easy editing externally
        elif should_encode(v, encode_types):
            v = json.dumps(v, skipkeys=True, sort_keys=True)

        params.append((ns, v))

    return params


def struct_from_params(
    struct: Union[object, type],
    params: Mapping[str, Any],
    *,
    encode_types=[Sequence, Mapping],
):
    """Reconstructs config struct from dict of parameters.

    If using struct class, it must not have any positional arguments.
    Use an instance of struct if aiming to merge configs (i.e apply changes.)

    Args:
        struct (Union[object, type]): Struct or instance of one.
        params (Mapping): Map of parameter names to values.
        encode_types (list, optional): List of types to decode. Defaults to [Sequence, Mapping].

    Returns:
        object: Instance of struct with values set based on params.
    """
    obj = struct() if isinstance(struct, type) else deepcopy(struct)

    for k, v in params.items():
        # ROS2 uses dot notation for parameter namespacing
        # See https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.declare_parameters
        path = deque(k.split("."))

        nested = obj
        while len(path) > 1:
            nested = getattr(nested, path.popleft())

        attr = getattr(nested, path[0])
        if should_encode(attr, encode_types):
            v = json.loads(v)

        setattr(nested, path[0], v)

    return obj
