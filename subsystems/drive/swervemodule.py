# standard python imports
import pint

# project imports
from constants import Drive as constants
from constants import unit as unit

# wpi imports
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6 import configs, signals, controls

class SwerveModule:
	def __init__(
		self,
		driveMotorId: int,
		turningMotorId: int,
		turningEncoderId: int,
		offset: pint.Quantity
	) -> None:
		"""Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

		:param driveMotorId:	  CAN ID for drive motor
		:param turningMotorId:	CAN ID for turning motor
		:param turningEncoderId:  CAN ID for turning encoder
		:param offset:  CANcoder offset
		"""

		# configure drive motor
		self.driveMotor = TalonFX(driveMotorId)

		drive_fx_cfg = configs.TalonFXConfiguration()
		slot0Config = drive_fx_cfg.slot0
		slot0Config.k_p = constants.kDrive_p
		slot0Config.k_i = constants.kDrive_i
		slot0Config.k_d = constants.kDrive_d
		slot0Config.k_v = constants.kDrive_v

		drive_fx_cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
		drive_fx_cfg.current_limits.stator_current_limit = 60
		drive_fx_cfg.current_limits.stator_current_limit_enable = True

		drive_fx_cfg.feedback.sensor_to_mechanism_ratio = constants.kDriveRatio

		self.driveMotor.configurator.apply(drive_fx_cfg)

		# configure absolute encoder
		self.turningEncoder = CANcoder(turningEncoderId)

		cc_cfg = configs.CANcoderConfiguration()
		cc_cfg.magnet_sensor.absolute_sensor_range = signals.AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
		cc_cfg.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
		cc_cfg.magnet_sensor.magnet_offset = offset.m_as("turn") # do this in Phoenix Tuner X instead
		self.turningEncoder.configurator.apply(cc_cfg)

		# configure turn motor
		self.turningMotor = TalonFX(turningMotorId)
		
		turn_fx_cfg = configs.TalonFXConfiguration()
		turn_fx_cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
		slot0Config = turn_fx_cfg.slot0
		slot0Config.k_p = constants.kTurn_p
		slot0Config.k_i = constants.kTurn_i
		slot0Config.k_d = constants.kTurn_d
		
		turn_fx_cfg.feedback.feedback_remote_sensor_id = turningEncoderId
		turn_fx_cfg.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
		turn_fx_cfg.feedback.sensor_to_mechanism_ratio = 1.0
		turn_fx_cfg.feedback.rotor_to_sensor_ratio = constants.kTurnRatio

		turn_fx_cfg.current_limits.stator_current_limit = 60
		turn_fx_cfg.current_limits.stator_current_limit_enable = True

		self.turningMotor.configurator.apply(turn_fx_cfg)

		self.position = wpimath.kinematics.SwerveModulePosition(0, wpimath.geometry.Rotation2d.fromDegrees(0))
		self.state = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(0))

	def getState(self) -> wpimath.kinematics.SwerveModuleState:
		"""Returns the current state of the module.

		:returns: The current state of the module.
		"""
		return self.state

	def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
		"""Returns the current position of the module.

		:returns: The current position of the module.
		"""
		return self.position
	
	def update(self) -> None:
		self.position = wpimath.kinematics.SwerveModulePosition(
			(self.driveMotor.get_position().refresh().value * unit.turn * constants.kWheelCircumference).m_as("meter"),
			wpimath.geometry.Rotation2d((self.turningMotor.get_position().refresh().value * unit.turn).m_as("radian")),
		)
		self.state = wpimath.kinematics.SwerveModuleState(
			(self.driveMotor.get_velocity().refresh().value * unit.turn / unit.second * constants.kWheelCircumference).m_as("meter / second"),
			wpimath.geometry.Rotation2d((self.turningMotor.get_position().refresh().value * unit.turn).m_as("radian")),
		)

	def setDesiredState(
		self, desiredState: wpimath.kinematics.SwerveModuleState
	) -> wpimath.kinematics.SwerveModuleState:
		"""Sets the desired state for the module.

		:param desiredState: Desired state with speed and angle.
		"""

		encoderRotation = wpimath.geometry.Rotation2d((self.turningMotor.get_position().refresh().value * unit.turn).m_as("radian"))

		# Optimize the reference state to avoid spinning further than 90 degrees
		state = wpimath.kinematics.SwerveModuleState.optimize(
			desiredState, encoderRotation
		)

		# Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
		# direction of travel that can occur when modules change directions. This results in smoother
		# driving.
		state.speed *= (state.angle - encoderRotation).cos()

		target_speed = (state.speed * unit.meter / unit.second / constants.kWheelCircumference).m_as("turn / second")
		target_angle = (state.angle.radians() * unit.radian).m_as("turn")

		drive_request = controls.VelocityVoltage(target_speed).with_slot(0)
		self.driveMotor.set_control(drive_request)

		turn_request = controls.PositionVoltage(target_angle).with_slot(0)
		self.turningMotor.set_control(turn_request)

		return state
