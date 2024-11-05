import math
from pint import UnitRegistry
import commands2.cmd as cmd

# real-world unit types
unit = UnitRegistry()

def makeCommand(func):
	def cmdFn(*args, **kwargs):
		return cmd.runOnce(lambda: func(*args, **kwargs))
	return cmdFn

class Global:
	# dashboard port used by the driver controller
	kDriverControllerPort = 0

class Drive:
	# motor module configs
	frontLeft = {
		"driveMotorId": 3,
		"turningMotorId": 4,
		"turningEncoderId": 10,
		"offset":-0.439 * unit.radian,
	}
	frontRight = {
		"driveMotorId": 7,
		"turningMotorId": 8,
		"turningEncoderId": 12,
		"offset":-1.592 * unit.radian,
	}
	backLeft = {
		"driveMotorId": 1,
		"turningMotorId": 2,
		"turningEncoderId": 9,
		"offset":-2.180 * unit.radian,
	}
	backRight = {
		"driveMotorId": 5,
		"turningMotorId": 6,
		"turningEncoderId": 11,
		"offset":3.073 * unit.radian,
	}
	# module parameters
	kWheelRadius = 2.0 * unit.inch
	kWheelCircumference = kWheelRadius * 2 * math.pi / unit.turn
	kEncoderResolution = 4096 #counts per rotation
	kModuleMaxAngularVelocity = math.pi * unit.radian / unit.second
	kModuleMaxAngularAcceleration = math.tau * unit.radian / unit.second / unit.second
	kTurnRatio = (150.0 / 7.0)
	kDriveRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
	kDrive_p = .01
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
	kChassisRadius = math.sqrt(kChassisWidth.m * kChassisWidth.m + kChassisLength.m * kChassisLength.m) * unit.inch
	kGyroId = 20

class Turntable:
	# motor ID as set in the firmware
	driveMotorId = 13
	# PIDv values for motor speed controll
	motorPID = {"p":.01,"i":0,"d":0,"v":.12}

class Spinner:
	# motor ID as set in the firmware
	driveMotorId = 14
	# PIDv values for motor speed controll
	motorPID = {"p":.01,"i":0,"d":0,"v":.12}
