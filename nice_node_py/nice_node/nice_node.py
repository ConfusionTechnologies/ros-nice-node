from copy import copy
from traceback import format_exc
from typing import Any, Callable, Dict, Generic, List, Sequence, Union

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.utilities import get_default_context
from ros2topic.api import get_msg_class
from std_msgs.msg import Header

from .types import *
from .utils import params_from_struct, should_update, struct_from_params

__all__ = ["NiceNode", "run"]

# TODO: make the global default RT_SUB_PROFILE & RT_PUB_PROFILE configurable

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

    def set_config(self, path: str, val: Any):
        """Set config value and update parameter.

        Args:
            path (str): _description_
            val (Any): _description_

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()
        self.set_parameters(Parameter())

    def _assert_key(self, key: str):
        """Ensure key is in config data structure."""
        try:
            if "." in key:
                v = self._default_cfg
                for k in key.split("."):
                    v = getattr(v, k)
            else:
                getattr(self._default_cfg, key)
        except AttributeError:
            assert False, f"Key {key} not found in config!"

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
                try:
                    func(self._get_param_dict())
                except TypeError:
                    func()  # func doesn't need changes dict
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
        for cleaning up things. Callback may optionally accept 1 positional parameter
        which is a dict mapping changed parameters to their values.

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
        for initializing things. Callback may optionally accept 1 positional parameter
        which is a dict mapping changed parameters to their values.

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

    # TODO: implementation of sub for message synchronization

    def sub(
        self,
        key: Union[str, list],
        msg_type: MsgType = None,
        qos: Union[QoSProfile, int] = RT_SUB_PROFILE,
        sync_queue_size: int = 30,
        sync_delay: float = 0.05,
    ):
        """Decorator to register subscription callback.

        If `msg_type` is None, this decorator will use `ros2topic.api` to get the
        message type and will block until successful.

        If `key` is a list, the function will subscribe to and synchronize multiple topics.
        In which case msg_type should either be a list or None.

        The default for `qos` is designed for realtime, high throughput applications.
        If greater reliability is wanted, set `qos` to an int, representing the queue
        depth, to get back the default reliable QoSProfile.

        Args:
            key (str): Config key containing the topic name to subscribe to.
            msg_type (MsgType, optional): Message type. Defaults to None.
            qos (Union[QoSProfile, int], optional): QosProfile. Defaults to RT_SUB_PROFILE.
            sync_queue_size (int): Queue size when syncing multiple topics. Defaults to 30.
            sync_delay (float): Max delay in seconds when syncing multiple topics. Defaults to 0.05.

        Returns:
            Callable: Decorator
        """
        is_multi = not isinstance(key, str) and isinstance(key, Sequence)
        keys = key if is_multi else [key]
        for k in keys:
            self._assert_key(k)

        def decorator(func: Callable[[MsgType], Any]):
            handles: List[Subscriber] = []

            @self.init(*keys)
            def sub(changes: Dict[str, Any]):
                topics: List[str] = [changes[k] for k in keys]
                if msg_type is None:
                    msg_types = [get_msg_class(self, t, blocking=True) for t in topics]
                else:
                    msg_types = msg_type if is_multi else [msg_type]

                for topic, typ in zip(topics, msg_types):
                    if typ is None:
                        typ = get_msg_class(self, topic, blocking=True)
                    handles.append(Subscriber(self, typ, topic, qos_profile=qos))

                if is_multi and len(handles) > 1:
                    ts = ApproximateTimeSynchronizer(
                        handles, sync_queue_size, sync_delay
                    )
                    ts.registerCallback(func)
                else:
                    handles[0].registerCallback(func)

            @self.clean(*keys)
            def unsub():
                nonlocal handles
                for h in handles:
                    self.destroy_subscription(h.sub)
                handles = []

            return func

        return decorator

    def sub_img(self, key: str, qos: Union[QoSProfile, int] = RT_SUB_PROFILE):
        """Decorator to register subscription callback to an Image/Compressed Image topic.

        Args:
            key (str): Config key containing the topic name to subscribe to.
            qos (Union[QoSProfile, int], optional): QosProfile. Defaults to RT_SUB_PROFILE.

        Returns:
            Callable: Decorator
        """
        self._assert_key(key)
        # NOTE: cvbridge is optional dependency, imported here only when needed
        # dont ask me why cvbridge decided to use a singleton structure instead
        # of storing their conversion maps as globals
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        bridge = CvBridge()

        def decorator(func: Callable[[np.ndarray, Header], Any]):
            @self.sub(key, None, qos)
            def wrapper(msg: Any):
                if isinstance(msg, Image):
                    img = bridge.imgmsg_to_cv2(msg, "bgr8")
                else:
                    img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                if 0 in img.shape:
                    self._logger.debug("Image has invalid shape!")
                    return
                func(img, msg.header)

            return func

        return decorator

    def pub(self, key: str, msg: Any, qos: QoSProfile = RT_PUB_PROFILE):
        """Publish message.

        The default for `qos` is designed for realtime, high throughput applications.
        If greater reliability is wanted, set `qos` to an int, representing the queue
        depth, to get back the default reliable QoSProfile.

        Args:
            key (str): Config key containing the topic name to publish to.
            msg (Any): Message to publish.
            qos (Union[QoSProfile, int], optional): QosProfile. Defaults to RT_PUB_PROFILE.
        """
        self._assert_key(key)
        publisher = self._publishers_cache.get(key, None)

        # lazily register callbacks to re-create publisher if topic name changes
        if publisher is None:

            @self.init(key)
            def create(changes: Dict[str, Any]):
                nonlocal publisher, msg, qos
                publisher = self.create_publisher(type(msg), changes[key], qos)
                self._publishers_cache[key] = publisher

            @self.clean(key)
            def clean():
                self.destroy_publisher(self._publishers_cache[key])

        publisher.publish(msg)

    def interval(self, key_or_rate: Union[str, int]):
        """Decorator to register callback that runs at an interval.

        Callback may optionally accept 1 positional parameter which is the delta
        since the last call in seconds.

        Args:
            key_or_rate (Union[str, int]): Either config key containing the rate or the rate.

        Returns:
            Callable: Decorator
        """

        from_cfg = isinstance(key_or_rate, str)
        if from_cfg:
            self._assert_key(key_or_rate)

        deps = [key_or_rate] if from_cfg else []

        def decorator(func: Callable):
            handle: Timer = None

            prev_time = -1

            def wrapper():
                nonlocal prev_time
                now = self.get_clock().now().nanoseconds
                delta = 0
                if prev_time != -1:
                    delta = now - prev_time
                prev_time = now
                try:
                    func(delta / 10 ** 9)
                except TypeError:
                    func()

            @self.init(*deps)
            def create(changes: Dict[str, Any]):
                nonlocal handle, func
                rate: int = changes[key_or_rate] if from_cfg else key_or_rate
                try:
                    handle = self.create_timer(1.0 / rate, wrapper)
                except ZeroDivisionError:
                    handle = None

            @self.clean(*deps)
            def clean():
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
