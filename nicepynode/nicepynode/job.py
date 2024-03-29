from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from traceback import format_exc
from typing import Generic, TypeVar

import yaml
from nicepynode.utils import dataclass_from_parameters
from rcl_interfaces.msg import ParameterDescriptor  # FloatingPointRange,
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty, Trigger

RESTART_SERVICE = "~/restart"
STOP_SERVICE = "~/stop"
CFG_PATH = Path("/data/node_cfg")


@dataclass
class JobCfg:
    """Default config for Job."""

    rate: float = 30.0
    """Rate in Hertz for job."""
    use_sim_time: bool = False
    """ROS Node param for using another clock; i.e. to run simulations."""


CT = TypeVar("CT", bound=JobCfg)
"""Generic for class that extends JobCfg."""


@dataclass
class Job(ABC, Generic[CT]):
    """Base class of common properties for Jobs."""

    # Although it is fields of Job, I still pass node, cfg, etc to the
    # function. It makes the functions clearer.
    # cfg vs params: params are from ROS Node, cfg is dataclass mirroring params

    node: Node = field(default_factory=Node)
    """Node to attach Job to."""
    ini_cfg: CT = field(default_factory=JobCfg)
    """Initial/default config for Job."""
    persist_cfg: bool = True
    """Whether changes to config should persist across restarts."""

    @property
    def cfg(self) -> CT:
        """Current (always updated) config of Job."""

        if self._cfg is None:
            self._cfg = dataclass_from_parameters(
                self.ini_cfg, {n: p.value for n, p in self.node._parameters.items()}
            )
        return self._cfg

    @abstractmethod
    def attach_params(self, node: Node, cfg: CT):
        """Attaches Job config to Node as parameters."""

        node.declare_parameter("rate", float(cfg.rate))
        node.set_descriptor(
            "rate",
            ParameterDescriptor(
                description="Rate in Hertz for job.",
                type=Parameter.Type.DOUBLE.value,
                # floating_point_range=[
                #     FloatingPointRange(from_value=0.0, to_value=None, step=0.0)
                # ],
            ),
        )

    @abstractmethod
    def attach_behaviour(self, node: Node, cfg: CT):
        """Attaches Job behaviours like pub-sub, services, etc to Node."""
        try:
            self._rate_timer = node.create_timer(1.0 / cfg.rate, self._rate_timer_cb)
        except ZeroDivisionError:
            self._rate_timer = None
        self._restart_srv = node.create_service(
            Trigger, RESTART_SERVICE, self._srv_restart
        )
        self._stop_srv = node.create_service(Empty, STOP_SERVICE, self._srv_crash)

    @abstractmethod
    def detach_behaviour(self, node: Node):
        """Detach & clean up Job behaviours attached to Node."""
        node.destroy_timer(self._rate_timer)

    @abstractmethod
    def on_params_change(self, node: Node, changes: dict):
        """Handle reconfiguration, returning whether restart is required, and
        raising if change should be rejected.

        Args:
            node (Node): ROS Node
            changes (dict): Map of parameter name to new value

        Returns:
            bool: Whether the change requires a restart.

        Raises:
            Exception: Reason change should be rejected.
        """
        return False

    def step(self, delta: float):
        """Called every cfg.rate. Delta is time in seconds since last call."""
        pass

    def params_to_cfg(self, params: list[Parameter]):
        """Convert list of Parameter to dict (Partial of JobCfg)."""
        return {p.name: p.value for p in params}

    def restart(self):
        """Restart Job, used for example to make config changes effective."""
        self.detach_behaviour(self.node)
        self.attach_behaviour(self.node, self.cfg)
        self.log.info("Restarted")

    def crash(self, reason=""):
        """Crashes the node after calling `detach_behaviour()`."""
        self.log.debug("Performing clean up before node crash.")
        self.detach_behaviour(self.node)
        assert False, reason

    def get_timestamp(self):
        """Shortcut for node.get_clock().now().to_msg()"""
        return self.node.get_clock().now().to_msg()

    def _rate_timer_cb(self):
        now = self.node.get_clock().now().nanoseconds
        try:
            delta = now - self._rate_timer_prev_time
        except AttributeError:
            delta = 0
        self._rate_timer_prev_time = now
        self.step(delta / 10 ** 9)

    def _param_change_cb(self, params: list[Parameter]):
        # ros2 only includes the params that were changed.
        changes = self.params_to_cfg(params)

        try:
            # will throw if config change is rejected
            restart = self.on_params_change(self.node, changes)
            # restart anyways if rate was changed
            restart = restart or "rate" in changes or "use_sim_time" in changes
            # apply changes NOW, parameters not updated yet by ROS
            self._cfg = dataclass_from_parameters(self._cfg, changes)

            if restart:
                self.log.debug(f"Config change requires restart.")
                self.restart()

            self.log.info(f"Config changed: {changes}")
            self._save_cfg(changes)
            return SetParametersResult(successful=True, reason="")
        except:
            exc = format_exc()
            self.log.error(f"Error applying {changes}:\n{exc}")
            return SetParametersResult(successful=False, reason=exc)

    def _srv_restart(self, req: Trigger.Request, res: Trigger.Response):
        """Service handler for restart service."""
        self.log.info("Restart service called.")
        try:
            self.restart()
        except:
            res.success = False
            res.message = format_exc()
            return res
        res.success = True
        return res

    def _srv_crash(self, req: Empty.Request, res: Empty.Response):
        """Service handler for stop service."""
        self.log.info("Stop service called.")

        @contextmanager
        def crash_after():
            try:
                yield
            finally:
                self.node.executor.create_task(
                    self.crash, reason="Stop service called."
                )

        with crash_after():
            return res

    def _save_cfg(self, changes: dict = {}):
        if not self.persist_cfg:
            return

        self._cfg_path.parent.mkdir(exist_ok=True, parents=True)

        params = {n: p.value for n, p in self.node._parameters.items()}
        params.update(changes)
        with open(self._cfg_path, "w") as f:
            yaml.safe_dump(params, f)

    # TODO: watch for config change in file
    def _load_cfg(self):
        if not self.persist_cfg:
            return

        if not self._cfg_path.exists():
            self._save_cfg()

        with open(self._cfg_path, "r") as f:
            obj: dict = yaml.safe_load(f)

        self.log.debug(f"Config loaded:\n{obj}")

        params = [Parameter(name, value=val) for name, val in obj.items()]
        # TODO: detect if param missing from config file; if so, replace with default value
        # default value = cfg struct + launch overrides
        result = self.node.set_parameters_atomically(params)
        self._cfg = None
        assert result.successful

    def __post_init__(self):
        """Attaches Job to Node."""
        self._cfg_path = (
            CFG_PATH / self.node.get_namespace()[1:] / f"{self.node.get_name()}.yaml"
        )

        self.log = self.node.get_logger()
        # current cfg based off node params for easier access
        # set to None whenever cfg needs to be recalculated
        # use self.cfg getter to get current cfg
        self._cfg = None

        # for the rate timer
        self._rate_timer = None

        # use self.ini_cfg to set params, which then sets self._cfg
        self.attach_params(self.node, self.ini_cfg)
        self._load_cfg()
        self.attach_behaviour(self.node, self.cfg)
        # NOTE: till all callbacks return True, ros won't actually change the
        # params. This can cause glitches such as stale config.
        self.node.add_on_set_parameters_callback(self._param_change_cb)
        # NOTE: there is no callback for parameters being set successfully, only
        # a topic broadcast. I refuse to have to add another listener for that which
        # is why we use the above callback.
