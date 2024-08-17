#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# standard imports
import math
from constants import Turntable as constants

# wpi imports
import wpilib
import commands2
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

# vendor imports
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6 import configs, signals, controls

class ShooterSubsystem(commands2.PIDSubsystem):
	def __init__(self) -> None:
		self.motor = TalonFX(constants.driveMotorId)

		motor_cfg = configs.TalonFXConfiguration()
		slot0Config = motor_cfg.slot0
		slot0Config.k_p = constants.motorPID["p"]
		slot0Config.k_i = constants.motorPID["i"]
		slot0Config.k_d = constants.motorPID["d"]
		slot0Config.k_v = constants.motorPID["v"]
		self.motor.configurator.apply(motor_cfg)

	# commands for spin in each direction