# project imports
import constants
from constants import makeCommand

# wpi imports
import commands2
import wpimath.controller
from wpilib import SmartDashboard

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6 import configs, controls, units


class Turntable(commands2.PIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                constants.Turntable.motorPID["p"],
                constants.Turntable.motorPID["i"],
                constants.Turntable.motorPID["d"],
            )
        )

        self.motor = TalonFX(
            constants.Turntable.driveMotorId,
            constants.Global.canivore
        )  # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html#phoenix6.hardware.core.core_talon_fx.CoreTalonFX

        motor_cfg = configs.TalonFXConfiguration()
        slot0Config = motor_cfg.slot0
        slot0Config.k_p = constants.Turntable.motorPID["p"]
        slot0Config.k_i = constants.Turntable.motorPID["i"]
        slot0Config.k_d = constants.Turntable.motorPID["d"]
        slot0Config.k_v = constants.Turntable.motorPID["v"]
        self.motor.configurator.apply(motor_cfg)

    @makeCommand
    def freespin(self, speed: units.rotations_per_second) -> None:
        turn_request = controls.VelocityVoltage(speed).with_slot(0)
        self.motor.set_control(turn_request)

    @makeCommand
    def turnto(self, position: units.degree) -> None:
        turn_request = controls.PositionDutyCycle(position / 360).with_slot(0)
        self.motor.set_control(turn_request)

    def turndeg(self, distance: units.degree) -> None:
        return self.turnto(distance + (360 * self.motor.get_position()._value))

    def periodic(self) -> None:
        super().periodic()
        SmartDashboard.putNumber("turntable", self.motor.get_position()._value)
