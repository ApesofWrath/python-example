# project imports
from robot import MyRobot
from constants import TunerConstants as constants

# wpi imports
from wpilib import DriverStation, RobotController
import wpilib.simulation as sim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath import units
from pyfrc.physics.core import PhysicsInterface

# vendor imports
from phoenix6.sim import ChassisReference
from phoenix6 import unmanaged


class PhysicsEngine:
    """
    Simulates a swerve robot
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller
        self.robot = robot

        # SWERVE INIT
        self.drivetrain = robot.container.robotDrive

        self.drive_ratio = constants._drive_gear_ratio
        self.turn_ratio = constants._steer_gear_ratio

        gearbox = DCMotor.krakenX60(1)

        backLeftTurn = sim.DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.01, self.turn_ratio), gearbox
        )
        backLeftTurn.setState(0, 0)
        self.drivetrain.backLeft.turningMotor.sim_state.set_raw_rotor_position(0)
        backRightTurn = sim.DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.01, self.turn_ratio), gearbox
        )
        backRightTurn.setState(0, 0)
        self.drivetrain.backRight.turningMotor.sim_state.set_raw_rotor_position(0)
        frontLeftTurn = sim.DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.01, self.turn_ratio), gearbox
        )
        frontLeftTurn.setState(0, 0)
        self.drivetrain.frontLeft.turningMotor.sim_state.set_raw_rotor_position(0)
        frontRightTurn = sim.DCMotorSim(
            LinearSystemId.DCMotorSystem(gearbox, 0.01, self.turn_ratio), gearbox
        )
        frontRightTurn.setState(0, 0)
        self.drivetrain.frontRight.turningMotor.sim_state.set_raw_rotor_position(0)

        self.swerve_sim_devices = [
            [
                self.drivetrain.backLeft.driveMotor.sim_state,
                sim.DCMotorSim(
                    LinearSystemId.DCMotorSystem(gearbox, 0.01, self.drive_ratio),
                    gearbox,
                ),
                self.drivetrain.backLeft.turningMotor.sim_state,
                backLeftTurn,
                self.drivetrain.backLeft.turningEncoder.sim_state,
            ],
            [
                self.drivetrain.backRight.driveMotor.sim_state,
                sim.DCMotorSim(
                    LinearSystemId.DCMotorSystem(gearbox, 0.01, self.drive_ratio),
                    gearbox,
                ),
                self.drivetrain.backRight.turningMotor.sim_state,
                backRightTurn,
                self.drivetrain.backRight.turningEncoder.sim_state,
            ],
            [
                self.drivetrain.frontLeft.driveMotor.sim_state,
                sim.DCMotorSim(
                    LinearSystemId.DCMotorSystem(gearbox, 0.01, self.drive_ratio),
                    gearbox,
                ),
                self.drivetrain.frontLeft.turningMotor.sim_state,
                frontLeftTurn,
                self.drivetrain.frontLeft.turningEncoder.sim_state,
            ],
            [
                self.drivetrain.frontRight.driveMotor.sim_state,
                sim.DCMotorSim(
                    LinearSystemId.DCMotorSystem(gearbox, 0.01, self.drive_ratio),
                    gearbox,
                ),
                self.drivetrain.frontRight.turningMotor.sim_state,
                frontRightTurn,
                self.drivetrain.frontRight.turningEncoder.sim_state,
            ],
        ]

        self.drivetrain.backLeft.turningMotor.sim_state.orientation = (
            ChassisReference.Clockwise_Positive
        )
        self.drivetrain.backRight.turningMotor.sim_state.orientation = (
            ChassisReference.Clockwise_Positive
        )
        self.drivetrain.frontLeft.turningMotor.sim_state.orientation = (
            ChassisReference.Clockwise_Positive
        )
        self.drivetrain.frontRight.turningMotor.sim_state.orientation = (
            ChassisReference.Clockwise_Positive
        )

        self.drivetrain.backLeft.turningEncoder.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        self.drivetrain.backRight.turningEncoder.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        self.drivetrain.frontLeft.turningEncoder.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        self.drivetrain.frontRight.turningEncoder.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )

        # GENERIC MOTOR INIT
        self.generic_motors = [
            robot.container.turntable.motor.sim_state,
            robot.container.spinner.motor.sim_state,
        ]
        for i in range(len(self.generic_motors)):
            self.generic_motors[i] = [
                self.generic_motors[i],
                sim.DCMotorSim(
                    LinearSystemId.DCMotorSystem(gearbox, 0.01, self.drive_ratio),
                    gearbox,
                ),
            ]

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # DRIVETRAIN SIM
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

        for corner in self.swerve_sim_devices:
            drive_fx, drive_motor, turn_fx, turn_motor, turn_encoder = corner

            drive_fx.set_supply_voltage(RobotController.getBatteryVoltage())
            drive_motor.setInputVoltage(drive_fx.motor_voltage)
            drive_motor.update(tm_diff)
            drive_fx.set_raw_rotor_position(units.radiansToRotations(drive_motor.getAngularPosition() * self.drive_ratio))
            drive_fx.set_rotor_velocity(units.radiansPerSecondToRotationsPerMinute(drive_motor.getAngularVelocity() * self.drive_ratio)*60)

            turn_fx.set_supply_voltage(RobotController.getBatteryVoltage())
            turn_motor.setInputVoltage(turn_fx.motor_voltage)
            turn_motor.update(tm_diff)
            turn_encoder.set_raw_position(units.radiansToRotations(turn_motor.getAngularPosition()))
            turn_encoder.set_velocity(units.radiansPerSecondToRotationsPerMinute(turn_motor.getAngularVelocity())*60)
            turn_fx.set_raw_rotor_position(units.radiansToRotations(turn_motor.getAngularPosition()))
            turn_fx.set_rotor_velocity(units.radiansPerSecondToRotationsPerMinute(turn_motor.getAngularVelocity()*self.turn_ratio)*60)

        self.drivetrain.gyro.sim_state.set_supply_voltage(
            RobotController.getBatteryVoltage()
        )

        # GENERIC SIM
        for motor in self.generic_motors:
            motor_fx, motor_sim = motor
            motor_fx.set_supply_voltage(RobotController.getBatteryVoltage())
            motor_sim.setInputVoltage(motor_fx.motor_voltage)
            motor_sim.update(tm_diff)
            motor_fx.set_raw_rotor_position(units.radiansToRotations(motor_sim.getAngularPosition() * self.drive_ratio))
            motor_fx.set_rotor_velocity(units.radiansPerSecondToRotationsPerMinute(motor_sim.getAngularVelocity() * self.drive_ratio)*60)