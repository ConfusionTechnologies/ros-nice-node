from contextlib import contextmanager
from copy import copy
from pathlib import Path
from traceback import format_exc
from typing import Any, Callable, Dict, Generic, List, Sequence, Union

import rclpy
import yaml
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.timer import Timer
from rclpy.utilities import get_default_context
from ros2topic.api import get_msg_class
from std_srvs.srv import Empty, Trigger

from .types import *
from .utils import params_from_struct, rosimg2numpy, should_update, struct_from_params

__all__ = ["NiceNode", "run"]

# TODO: make the global default RT_SUB_PROFILE & RT_PUB_PROFILE configurable
# TODO: expose callback groups which are useful for multi-threaded executors,
# see: https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
# TODO: hang detection for everything

# Realtime Profile: don't bog down publisher when model is slow
RT_SUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_SUB_PROFILE.depth = 0

# Realtime Profile: don't wait for slow subscribers
RT_PUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_PUB_PROFILE.depth = 0

RESTART_SERVICE = "~/restart"
STOP_SERVICE = "~/stop"
CFG_PATH = Path("/data/node_cfg")


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

    def __init__(self, node_name: str = ..., persist_cfg: bool = True, **kwargs):
        """See https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node for all arguments.

        NiceNode inherits from rclpy.Node. If `node_name` is `...`, a random name will
        be generated. Do note that using a random name means the node config will
        not persist across restarts. If `persist_cfg` is True, config will be saved
        and reloaded from a yaml file.
        """

        # Init default context if user doesn't provide custom context & default
        # context hasn't init yet, preventing user from needing to do so themselves
        if kwargs.get("context", None) is None and not get_default_context().ok():
            rclpy.init()

        if node_name is None:
            node_name = f"{self.__class__.__name__}-{hex(id(self))[2:]}"

        super(NiceNode, self).__init__(node_name, **kwargs)

        self.persist_cfg = persist_cfg
        self._cfg_path = CFG_PATH / self.get_namespace()[1:] / f"{self.get_name()}.yaml"

        self._init_callbacks: CallbackListType = []
        self._clean_callbacks: CallbackListType = []
        # I refuse to search the internal self._publishers each time for topic
        self._publishers_cache: Dict[str, Publisher] = {}

        # For restart and stop services; Doesn't need clean up as they always exist.
        self.create_service(Trigger, RESTART_SERVICE, self._srv_restart)
        self.create_service(Empty, STOP_SERVICE, self._srv_stop)

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

        self._default_cfg = cfg()
        self._cfg = None

        params = self.declare_parameters("", params_from_struct(cfg, **kwargs))
        self._load_config()  # must be before param callback is added
        self.add_on_set_parameters_callback(self._callback_params_changed)
        self._logger.debug(f"Config declared:\n{self.cfg}")
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

    def restart(self):
        """Restarts the node by calling all init & clean callbacks."""
        # self._get_param_dict() isn't cached; but what if callback changes param
        # during restart? hence call it each time instead of reusing.
        for func, _ in self._clean_callbacks:
            func(self._get_param_dict())
        for func, _ in self._init_callbacks:
            func(self._get_param_dict())
        self._logger.info("Node restarted.")

    def crash(self, reason=""):
        """Crashes the node after calling all clean callbacks.

        Args:
            reason (Any, optional): Reason for crash. Defaults to "".
        """
        self._logger.debug("Performing clean up before node crash.")
        for func, _ in self._clean_callbacks:
            try:
                # TODO: hang detection for clean up
                func(self._get_param_dict())
            except:
                self._logger.error(f"Error during clean up:\n{format_exc()}")
        self.destroy_node()
        assert False, reason

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

            self._logger.debug(f"Config change successful.")
            self._save_config(changes)
            return SetParametersResult(successful=True, reason="")
        except:
            # Should crash because node may be in invalid state?
            exc = format_exc()
            self._logger.error(f"Error applying changes:\n{exc}")
            return SetParametersResult(successful=False, reason=exc)

    def _srv_restart(self, req: Trigger.Request, res: Trigger.Response):
        """Service handler for restart service."""
        self._logger.info("Restart service called.")
        try:
            self.restart()
        except:
            # Should crash because node may be in invalid state?
            res.success = False
            res.message = format_exc()
            return res
        res.success = True
        return res

    def _srv_stop(self, req: Empty.Request, res: Empty.Response):
        """Service handler for stop service."""
        self._logger.info("Stop service called.")

        @contextmanager
        def crash_after():
            try:
                yield
            finally:
                self.executor.create_task(self.crash, reason="Stop service called.")

        with crash_after():
            return res

    # TODO: is there a use case for remove_callback?
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

            def wrapper(changes: dict):
                try:
                    func(changes)
                except TypeError:
                    func()  # func doesn't need changes dict

            arr.append((wrapper, deps))
            if now:
                # call func now with all parameters as "changes"
                wrapper(self._get_param_dict())

            return func

        return decorator

    def _get_param_dict(self) -> Dict[str, Any]:
        """Get current parameters as dict."""
        params = {n: p.value for n, p in self._parameters.items()}
        params.pop("use_sim_time", None)  # remove predeclared ROS2 parameter
        return params

    def _save_config(self, changes: dict = {}):
        """Save current node parameters to `self._cfg_path`."""
        if not self.persist_cfg:
            return

        self._cfg_path.parent.mkdir(exist_ok=True, parents=True)

        params = self._get_param_dict()
        params.update(changes)
        with open(self._cfg_path, "w") as f:
            yaml.safe_dump(params, f)

    # TODO: watch for config change in file
    def _load_config(self):
        """Load and set node parameters from `self._cfg_path` file."""
        if not self.persist_cfg:
            return

        if not self._cfg_path.exists():
            self._save_config()

        with open(self._cfg_path, "r") as f:
            obj: Dict[str, Any] = yaml.safe_load(f)

        params = [Parameter(name, value=val) for name, val in obj.items()]
        # TODO: detect if param missing from config file; if so, replace with default value
        # default value = cfg struct + launch overrides
        result = self.set_parameters_atomically(params)
        self._cfg = None
        assert result.successful

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

    def sub(
        self,
        key: Union[str, list],
        msg_type: MsgType = None,
        qos: Union[QoSProfile, int] = RT_SUB_PROFILE,
        sync_queue_size: int = 30,
        sync_delay: float = 0.05,
        convert_img: bool = True,
        convert_img_enc: str = "bgr8",
    ):
        """Decorator to register subscription callback.

        If `msg_type` is None, this decorator will use `ros2topic.api` to get the
        message type and will block until successful.

        If `key` is a list, the function will subscribe to and synchronize multiple topics.
        In which case msg_type should either be a list or None.

        The default for `qos` is designed for realtime, high throughput applications.
        If greater reliability is wanted, set `qos` to an int, representing the queue
        depth, to get back the default reliable QoSProfile.

        For `convert_img_enc`, see https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
        for available encodings.

        Args:
            key (str): Config key containing the topic name to subscribe to.
            msg_type (MsgType, optional): Message type. Defaults to None.
            qos (Union[QoSProfile, int], optional): QosProfile. Defaults to RT_SUB_PROFILE.
            sync_queue_size (int, optional): Queue size when syncing multiple topics. Defaults to 30.
            sync_delay (float, optional): Max delay in seconds when syncing multiple topics. Defaults to 0.05.
            convert_img (bool, optional): Auto-convert CompressedImage/Image to numpy array. Defaults to True.
            convert_img_enc (str, optional): Encoding for numpy array. Defaults to "bgr8".

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
                        # TODO: do periodically instead of blocking to prevent hang
                        typ = get_msg_class(self, topic, blocking=True)
                    handles.append(Subscriber(self, typ, topic, qos_profile=qos))

                def wrapper(*msgs: Any):
                    args = []
                    header = None
                    for msg in msgs:
                        if hasattr(msg, "header"):
                            header = msg.header
                        args.append(
                            rosimg2numpy(msg, convert_img_enc) if convert_img else msg
                        )
                    try:
                        func(header=header, *args)
                    except TypeError:
                        func(*args)

                if is_multi and len(handles) > 1:
                    ts = ApproximateTimeSynchronizer(
                        handles, sync_queue_size, sync_delay
                    )
                    ts.registerCallback(wrapper)
                else:
                    handles[0].registerCallback(wrapper)

            @self.clean(*keys)
            def unsub():
                nonlocal handles
                for h in handles:
                    self.destroy_subscription(h.sub)
                handles = []

            return func

        return decorator

    def sub_img(self, key: str, qos: Union[QoSProfile, int] = RT_SUB_PROFILE):
        """DEPRECATED: `NiceNode.sub()` can now convert images!

        Decorator to register subscription callback to an Image/Compressed Image topic.
        Image will be BGR8, similar to `cv2.imread()`.

        Args:
            key (str): Config key containing the topic name to subscribe to.
            qos (Union[QoSProfile, int], optional): QosProfile. Defaults to RT_SUB_PROFILE.

        Returns:
            Callable: Decorator
        """
        return self.sub(key, qos=qos)

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
                    func(delta / 10**9)
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

    def srv(self):
        raise NotImplementedError

    def call_srv(self):
        raise NotImplementedError

    def action(self):
        raise NotImplementedError

    def call_action(self):
        raise NotImplementedError


def run(node: NiceNode):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.crash()
        except:
            pass  # suppress error
    except:
        node.crash()
