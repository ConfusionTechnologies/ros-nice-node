from copy import copy
from traceback import format_exc
from typing import Any, Callable, Dict, Generic, List, Union

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.utilities import get_default_context
from ros2topic.api import get_msg_class

from .types import *
from .utils import params_from_struct, should_update, struct_from_params

__all__ = ["NiceNode", "run"]


# Realtime Profile: don't bog down publisher when model is slow
RT_SUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_SUB_PROFILE.depth = 0

# Realtime Profile: don't wait for slow subscribers
RT_PUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_PUB_PROFILE.depth = 0


class NiceNode(Node, Generic[CfgType]):
    @property
    def cfg(self) -> CfgType:
        """Current config for node based on parameters."""
        try:
            # NOTE: set self._cfg to None to recalculate config from parameters
            if self._cfg is None:
                self._cfg = struct_from_params(
                    self._default_cfg, self._get_param_dict()
                )
            return self._cfg
        except AttributeError:
            # NOTE: declare_config() has to be called first
            exc = format_exc()
            raise RuntimeError(f"Config has not been declared yet!\n{exc}")

    def __init__(self, node_name: str = ..., **kwargs):
        """See https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node for all arguments.

        NiceNode inherits from rclpy.Node. If `node_name` is `...`, a random name will
        be generated.
        """

        # Init default context if user doesn't provide custom context & default
        # context hasn't init yet, preventing user from needing to do so themselves
        if kwargs.get("context", None) is None and not get_default_context().ok():
            rclpy.init()

        if node_name is None:
            node_name = f"{self.__class__.__name__}-{hex(id(self))[2:]}"

        super(NiceNode, self).__init__(node_name, **kwargs)

        self._init_callbacks: CallbackListType = []
        self._clean_callbacks: CallbackListType = []
        # I refuse to search the internal self._publishers each time for topic
        self._publishers_cache: Dict[str, Publisher] = {}

    def declare_config(self, cfg: CfgType, **kwargs):
        """Declare node parameters from config.

        See keyword arguments of `nice_node.utils.params_from_struct()`.

        Args:
            cfg (CfgType): Config struct.

        Returns:
            List[Parameter]: Return value of `node.declare_parameters()`.
        """
        # NOTE: resetting the parameters like this is probably a bad idea
        if hasattr(self, "_default_cfg"):
            for p in self._parameters.keys():
                try:
                    self.undeclare_parameter(p)
                except:
                    pass

        self._default_cfg = cfg
        self._cfg = None

        params = self.declare_parameters("", params_from_struct(cfg, **kwargs))
        self.add_on_set_parameters_callback(self._callback_params_changed)
        self._logger.info(f"Config declared:\n{self.cfg}")
        return params

    def set_cfg(self, path: str, val: Any):
        """Set config value and update parameter.

        Args:
            path (str): _description_
            val (Any): _description_

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()
        self.set_parameters(Parameter())

    def _callback_params_changed(self, params: List[Parameter]):
        """Callback attached to `node.add_on_set_parameters_callback()`."""
        try:
            # only includes the params that were changed
            changes: Dict[str, Any] = {p.name: p.value for p in params}
            self._logger.info(f"Config change:\n{changes}")
            changes.pop("use_sim_time", None)  # remove predeclared ROS2 parameter

            # trigger clean before cfg update
            for func, deps in self._clean_callbacks:
                if should_update(deps, changes.keys()):
                    func(changes)

            # NOTE: This callback occurs before the parameters are actually set.
            # The official way to be notified when the parameters are set is through
            # the /parameter_events topic. However, that is jank and the helper class
            # for it hasn't been released in rclpy yet.
            # See: https://github.com/ros2/rclpy/pull/959
            # Assume changes are correct & apply NOW; use self.cfg to ensure it exists
            self._cfg = struct_from_params(self.cfg, changes)

            # trigger init after cfg update
            for func, deps in self._init_callbacks:
                if should_update(deps, changes.keys()):
                    func(changes)

            self._logger.info(f"Config change successful")
            return SetParametersResult(successful=True, reason="")
        except:
            exc = format_exc()
            self._logger.info(f"Error applying changes:\n{exc}")
            return SetParametersResult(successful=False, reason=exc)

    def _add_callback(self, arr: CallbackListType, now: bool, *deps: str):
        """Internal function to add a callback to NiceNode via a decorator."""

        def decorator(func: CallbackType):
            nonlocal deps
            # standardize
            if len(deps) == 0:
                deps = ...  # always rerun
            elif deps[0] is None:
                deps = None  # don't rerun
            else:
                deps = set(deps)
            arr.append((func, deps))
            if now:
                # call func now with all parameters as "changes"
                func(self._get_param_dict())
            return func

        return decorator

    def _get_param_dict(self) -> Dict[str, Any]:
        """Get current parameters as dict."""
        params = {n: p.value for n, p in self._parameters.items()}
        params.pop("use_sim_time", None)  # remove predeclared ROS2 parameter
        return params

    def init(self, *deps: str, now=True):
        """Decorator to register callback to run after parameter/config change.

        Typically used to initialize things based on the config. See `NiceNode.clean()`
        for cleaning up things.

        Arguments are the config keys that would trigger the function to re-run.
        If None is provided as the argument, the callback will not re-run unless
        the node is ordered to restart. If no arguments are provided, the callback
        will re-run whenever the config changes.

        Returns:
            Callable: Decorator
        """
        return self._add_callback(self._init_callbacks, now, *deps)

    def clean(self, *deps: str, now=False):
        """Decorator to register callback to run before parameter/config change.

        Typically used to clean up things based on the config. See `NiceNode.init()`
        for initializing things.

        Arguments are the config keys that would trigger the function to re-run.
        If None is provided as the argument, the callback will not re-run unless
        the node is ordered to restart. If no arguments are provided, the callback
        will re-run whenever the config changes.

        BTW, if you use `@clean(None)`, it will literally never run. So don't.

        Returns:
            Callable: Decorator
        """
        return self._add_callback(self._clean_callbacks, now, *deps)

    # The decision has been made that subscribe/publish won't auto-declare config options
    # due to the added complexity of handling a dynamically typed config object.
    # Either we sacrifice simple names via namespacing, or we risk name collisions.

    # TODO: subscribe & unsubsribe etc functions are basically specific implementations
    # of init and clean with specific dependencies

    # TODO: implementation of sub for message synchronization

    def sub(self, key: str, msg_type: MsgType = None, qos: Any = RT_SUB_PROFILE):
        """Decorator to register subscription callback.

        If `msg_type` is None, this decorator will use `ros2topic.api` to get the
        message type and will block until successful.

        Args:
            key (str): Config key containing the topic name to subscribe to.
            msg_type (MsgType, optional): Message type. Defaults to None.
            qos (Any, optional): QoS level. Defaults to 5.

        Returns:
            Callable: Decorator
        """
        assert hasattr(self._default_cfg, key), f"Key {key} not found in config!"

        def decorator(func: Callable[[MsgType], Any]):
            handle: Subscription = None

            @self.init(key)
            def sub(changes: Dict[str, Any]):
                nonlocal msg_type, qos, handle, func
                topic: str = changes[key]
                if msg_type is None:
                    msg_type = get_msg_class(self, topic, blocking=True)
                handle = self.create_subscription(msg_type, topic, func, qos)

            @self.clean(key)
            def unsub(_):
                nonlocal handle
                self.destroy_subscription(handle)

            return func

        return decorator

    def pub(self, key: str, msg: Any, qos: Any = RT_PUB_PROFILE):
        """Publish message.

        Args:
            key (str): Config key containing the topic name to publish to.
            msg (Any): Message to publish.
            qos (Any, optional): QoS level. Defaults to 5.
        """
        assert hasattr(self._default_cfg, key), f"Key {key} not found in config!"
        publisher = self._publishers_cache.get(key, None)

        # lazily register callbacks to re-create publisher if topic name changes
        if publisher is None:

            @self.init(key)
            def create(changes: Dict[str, Any]):
                nonlocal publisher, msg, qos
                publisher = self.create_publisher(type(msg), changes[key], qos)
                self._publishers_cache[key] = publisher

            @self.clean(key)
            def clean(_):
                self.destroy_publisher(self._publishers_cache[key])

        publisher.publish(msg)

    def interval(self, key_or_rate: Union[str, int]):
        """Decorator to register callback that runs at an interval.

        Args:
            key_or_rate (Union[str, int]): Either config key containing the rate or the rate.

        Returns:
            Callable: Decorator
        """

        # TODO: timer delta given to callback
        from_cfg = isinstance(key_or_rate, str)
        assert not from_cfg or hasattr(
            self._default_cfg, key_or_rate
        ), f"Key {key_or_rate} not found in config!"

        deps = [key_or_rate] if from_cfg else []

        def decorator(func: Callable):
            handle: Timer = None

            @self.init(*deps)
            def create(changes: Dict[str, Any]):
                nonlocal handle, func
                rate: int = changes[key_or_rate] if from_cfg else key_or_rate
                try:
                    handle = self.create_timer(1.0 / rate, func)
                except ZeroDivisionError:
                    handle = None

            @self.clean(*deps)
            def clean(_):
                nonlocal handle
                if handle:
                    self.destroy_timer(handle)

            return func

        return decorator


def run(node: NiceNode):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
