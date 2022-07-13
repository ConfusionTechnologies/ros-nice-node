from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Generic, TypeVar
from dataclasses import dataclass, field, replace
from traceback import format_exc

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    # FloatingPointRange,
    SetParametersResult,
)


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

    @property
    def cfg(self) -> CT:
        """Current (always updated) config of Job."""

        if self._cfg is None:
            self._cfg = type(self.ini_cfg)(
                **{n: p.value for n, p in self.node._parameters.items()}
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

    @abstractmethod
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
            restart = restart or "rate" in changes
            # apply changes NOW, parameters not updated yet by ROS
            self._cfg = replace(self._cfg, **changes)

            if restart:
                self.log.debug(f"Config change requires restart.")
                self.restart()

            self.log.debug(f"Config changed: {changes}")
            return SetParametersResult(successful=True, reason="")
        except:
            exc = format_exc()
            self.log.error(f"Error applying {changes}:\n{exc}")
            return SetParametersResult(successful=False, reason=exc)

    def __post_init__(self):
        """Attaches Job to Node."""

        self.log = self.node.get_logger()
        # current cfg based off node params for easier access
        # set to None whenever cfg needs to be recalculated
        # use self.cfg getter to get current cfg
        self._cfg = None

        # for the rate timer
        self._rate_timer = None

        # use self.ini_cfg to set params, which then sets self._cfg
        self.attach_params(self.node, self.ini_cfg)
        # TODO: check that params can be successfully loaded from ROS save file
        self.attach_behaviour(self.node, self.cfg)
        # NOTE: till all callbacks return True, ros won't actually change the
        # params. This can cause glitches such as stale config.
        self.node.add_on_set_parameters_callback(self._param_change_cb)
