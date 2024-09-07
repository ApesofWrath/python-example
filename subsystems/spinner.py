# standard imports
import math
from constants import Spinner as constants
from enum import Enum

# wpi imports
import wpilib
import commands2
import wpimath.controller
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6 import configs, signals, controls

class Spinner(commands2.PIDSubsystem):
	def __init__(self) -> None:
		super().__init__(
			wpimath.controller.PIDController(
				constants.motorPID["p"],
				constants.motorPID["i"],
				constants.motorPID["d"]
			)
		)

		self.motor = TalonFX(constants.driveMotorId) # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html#phoenix6.hardware.core.core_talon_fx.CoreTalonFX

		motor_cfg = configs.TalonFXConfiguration()
		slot0Config = motor_cfg.slot0
		slot0Config.k_p = constants.motorPID["p"]
		slot0Config.k_i = constants.motorPID["i"]
		slot0Config.k_d = constants.motorPID["d"]
		slot0Config.k_v = constants.motorPID["v"]
		self.motor.configurator.apply(motor_cfg)

		self.states = Enum("States", ["SLOW","FAST"]) # type: ignore
		self.state = self.states.SLOW

	def slow(self) -> None:
		self.state = self.states.SLOW

	def fast(self) -> None:
		self.state = self.states.FAST

	def periodic(self) -> None:
		super().periodic()
		match self.state:
			case self.states.SLOW:
				turn_request = controls.VelocityVoltage(3).with_slot(0)
			case self.states.FAST:
				turn_request = controls.VelocityVoltage(20).with_slot(0)
		self.motor.set_control(turn_request)
		SmartDashboard.putNumber("spinner", self.motor.get_position()._value)