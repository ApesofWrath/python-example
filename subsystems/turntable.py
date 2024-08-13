#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# standard python imports
import math

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

driveMotorId = 13 # should be unallocated
motorPID = {"p":.01,"i":0,"d":0,"v":.12}

class ShooterSubsystem(commands2.PIDSubsystem):
	def __init__(self) -> None:
		self.motor = TalonFX(driveMotorId)

		motor_cfg = configs.TalonFXConfiguration()
		slot0Config = motor_cfg.slot0
		slot0Config.k_p = motorPID["p"]
		slot0Config.k_i = motorPID["i"]
		slot0Config.k_d = motorPID["d"]
		slot0Config.k_v = motorPID["v"]
		self.motor.configurator.apply(motor_cfg)

		# encoder: is it integrated
		# how else does the motor need to be configured
	
	# commands for spin in each direction