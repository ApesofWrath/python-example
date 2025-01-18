# standard imports
from constants import Spinner as constants
from enum import Enum

# wpi imports
import commands2
import commands2.cmd as cmd
import wpimath.controller
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import configs, controls


class Spinner(commands2.PIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                constants.motorPID["p"],
                constants.motorPID["i"],
                constants.motorPID["d"],
            )
        )

        self.motor = TalonFX(
            constants.driveMotorId
        )  # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html#phoenix6.hardware.core.core_talon_fx.CoreTalonFX

        motor_cfg = configs.TalonFXConfiguration()
        slot0Config = motor_cfg.slot0
        slot0Config.k_p = constants.motorPID["p"]
        slot0Config.k_i = constants.motorPID["i"]
        slot0Config.k_d = constants.motorPID["d"]
        slot0Config.k_v = constants.motorPID["v"]
        self.motor.configurator.apply(motor_cfg)

        self.states = Enum("States", ["ON", "OFF"])  # type: ignore
        self.state = self.states.OFF

    def setState(self, state) -> None:
        self.state = state

    def on(self) -> commands2.Command:
        return cmd.runOnce(lambda: self.setState(self.states.ON))

    def off(self) -> commands2.Command:
        return cmd.runOnce(lambda: self.setState(self.states.OFF))

    def periodic(self) -> None:
        super().periodic()
        match self.state:
            case self.states.ON:
                turn_request = controls.VelocityVoltage(3).with_slot(0)
            case self.states.OFF:
                turn_request = controls.VelocityVoltage(0).with_slot(0)
        self.motor.set_control(turn_request)
        SmartDashboard.putNumber("spinner", self.motor.get_position()._value)
