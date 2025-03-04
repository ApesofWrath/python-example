import math

from phoenix6 import CANBus, configs, signals, swerve, units
from wpimath.units import inchesToMeters, rotationsToRadians, degreesToRadians
import commands2.cmd as cmd
from wpimath.geometry import Pose2d, Transform2d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pint import UnitRegistry
unit = UnitRegistry()


def makeCommand(func):
    def cmdFn(*args, **kwargs):
        return cmd.runOnce(lambda: func(*args, **kwargs))

    return cmdFn


class Drive:
    # module parameters
    kWheelRadius = 2.0 * unit.inch
    kWheelCircumference = kWheelRadius * 2 * math.pi / unit.turn
    kEncoderResolution = 4096  # counts per rotation
    kModuleMaxAngularVelocity = math.pi * unit.radian / unit.second
    kModuleMaxAngularAcceleration = math.tau * unit.radian / unit.second / unit.second
    kTurnRatio = 150.0 / 7.0
    kDriveRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    kDrive_p = 0.01
    kDrive_i = 0
    kDrive_d = 0
    kDrive_v = 12.0 / (100.0 / kDriveRatio)
    kTurn_p = 40
    kTurn_i = 0
    kTurn_d = 0
    # drivetrain paramaters
    kMaxSpeed = 3.0 * unit.meter / unit.second  # 3 meters per second
    kMaxAngularSpeed = math.pi * unit.radian / unit.second  # 1/2 rotation per second
    kChassisWidth = 28.0 * unit.inch
    kChassisLength = 28.0 * unit.inch
    kChassisRadius = (
        math.sqrt(
            kChassisWidth.m * kChassisWidth.m + kChassisLength.m * kChassisLength.m
        )
        * unit.inch
    )

class Limelight:
    kGyroId = 20
    kLimelightHostnames = [ "limelight-wwdkd", "limelight-jonkler", "limelight-moist" ]

    kAlignmentTargets = { id: AprilTagFieldLayout().loadField(AprilTagField.kDefaultField).getTagPose(id).toPose2d().transformBy(Transform2d(.5,0,math.pi)) for id in list(range(6,12))+list(range(17,23)) }

    class precise:
        move_p = 2
        spin_p = 0.75
        xy_tolerance = 0.01
        theta_tolerance = 0.5

class Turntable:
    # motor ID as set in the firmware
    driveMotorId = 13
    # PIDv values for motor speed controll
    motorPID = {"p": 0.01, "i": 0, "d": 0, "v": 0.12}

class Spinner:
    # motor ID as set in the firmware
    driveMotorId = 14
    # PIDv values for motor speed controll
    motorPID = {"p": 0.01, "i": 0, "d": 0, "v": 0.12}

class TunerConstants:
    """
    Generated by the Tuner X Swerve Project Generator
    https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    """

    # Both sets of gains need to be tuned to your individual robot

    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(100)
        .with_k_i(0)
        .with_k_d(0.5)
        .with_k_s(0.1)
        .with_k_v(2.66)
        .with_k_a(0)
        .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    )
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.1)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0)
        .with_k_v(0.124)
    )

    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The type of motor used for the drive motor
    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    # The type of motor used for the drive motor
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot
    _slip_current: units.ampere = 120.0

    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    _drive_initial_configs = configs.TalonFXConfiguration()
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        # Swerve azimuth does not require much torque output, so we can set a relatively low
        # stator current limit to help avoid brownouts without impacting performance.
        .with_stator_current_limit(60).with_stator_current_limit_enable(True)
    )
    _encoder_initial_configs = configs.CANcoderConfiguration()
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs: configs.Pigeon2Configuration | None = None

    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus = CANBus("", "./logs/example.hoot")

    # Theoretical free speed (m/s) at 12 V applied output;
    # This needs to be tuned to your individual robot
    speed_at_12_volts: units.meters_per_second = 4.73

    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    _couple_ratio = 3.5714285714285716

    _drive_gear_ratio = 6.746031746031747
    _steer_gear_ratio = 21.428571428571427
    _wheel_radius: units.meter = inchesToMeters(2)

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 20

    # These are only used for simulation
    _steer_inertia: units.kilogram_square_meter = 0.01
    _drive_inertia: units.kilogram_square_meter = 0.01
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage: units.volt = 0.2
    _drive_friction_voltage: units.volt = 0.2

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator: swerve.SwerveModuleConstantsFactory[configs.TalonFXConfiguration, configs.TalonFXConfiguration, configs.CANcoderConfiguration] = (
        swerve.SwerveModuleConstantsFactory()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
        .with_steer_inertia(_steer_inertia)
        .with_drive_inertia(_drive_inertia)
        .with_steer_friction_voltage(_steer_friction_voltage)
        .with_drive_friction_voltage(_drive_friction_voltage)
    )


    # Front Left
    _front_left_drive_motor_id = 3
    _front_left_steer_motor_id = 4
    _front_left_encoder_id = 10
    _front_left_encoder_offset: units.rotation = 0.431884765625
    _front_left_steer_motor_inverted = True
    _front_left_encoder_inverted = False

    _front_left_x_pos: units.meter = inchesToMeters(14)
    _front_left_y_pos: units.meter = inchesToMeters(14)

    # Front Right
    _front_right_drive_motor_id = 7
    _front_right_steer_motor_id = 8
    _front_right_encoder_id = 12
    _front_right_encoder_offset: units.rotation = -0.236572265625
    _front_right_steer_motor_inverted = True
    _front_right_encoder_inverted = False

    _front_right_x_pos: units.meter = inchesToMeters(14)
    _front_right_y_pos: units.meter = inchesToMeters(-14)

    # Back Left
    _back_left_drive_motor_id = 1
    _back_left_steer_motor_id = 2
    _back_left_encoder_id = 9
    _back_left_encoder_offset: units.rotation = 0.14599609375
    _back_left_steer_motor_inverted = True
    _back_left_encoder_inverted = False

    _back_left_x_pos: units.meter = inchesToMeters(-14)
    _back_left_y_pos: units.meter = inchesToMeters(14)

    # Back Right
    _back_right_drive_motor_id = 5
    _back_right_steer_motor_id = 6
    _back_right_encoder_id = 11
    _back_right_encoder_offset: units.rotation = 0.30517578125
    _back_right_steer_motor_inverted = True
    _back_right_encoder_inverted = False

    _back_right_x_pos: units.meter = inchesToMeters(-14)
    _back_right_y_pos: units.meter = inchesToMeters(-14)


    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )
    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )
    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )
    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

class Global:
    # dashboard port used by the driver controller
    kDriverControllerPort = 0
    # TODO: remove redundancy, these are already in TunerConstants
    max_speed = TunerConstants.speed_at_12_volts # desired top speed
    break_speed_mul = 0.5
    max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity
    canivore = "0A58A9B5463847532020204B17130DFF"