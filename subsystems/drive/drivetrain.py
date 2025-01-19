# standard python imports
import random

import commands2
import ntcore
import wpimath.geometry
import wpimath.kinematics
# auton imports
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
# vendor imports
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import DriverStation
# wpi imports
from wpilib import SmartDashboard, Field2d
from wpimath import estimator
from wpimath.geometry import Rotation2d, Pose2d, Twist2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

from constants import Drive as constants
# project imports
from subsystems.drive.swervemodule import SwerveModule


# TODO: https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/
class Drivetrain(commands2.Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self, isReal: bool = True) -> None:
        super().__init__()
        self._isReal = isReal
        self._simPose = Pose2d()

        self.frontLeftLocation = wpimath.geometry.Translation2d(
            (constants.kChassisWidth / 2.0).m_as("meter"),
            (constants.kChassisLength / 2.0).m_as("meter"),
        )
        self.frontRightLocation = wpimath.geometry.Translation2d(
            (constants.kChassisWidth / 2.0).m_as("meter"),
            (-constants.kChassisLength / 2.0).m_as("meter"),
        )
        self.backLeftLocation = wpimath.geometry.Translation2d(
            (-constants.kChassisWidth / 2.0).m_as("meter"),
            (constants.kChassisLength / 2.0).m_as("meter"),
        )
        self.backRightLocation = wpimath.geometry.Translation2d(
            (-constants.kChassisWidth / 2.0).m_as("meter"),
            (-constants.kChassisLength / 2.0).m_as("meter"),
        )

        self.frontLeft = SwerveModule(**constants.frontLeft)
        self.frontRight = SwerveModule(**constants.frontRight)
        self.backLeft = SwerveModule(**constants.backLeft)
        self.backRight = SwerveModule(**constants.backRight)

        self.gyro = Pigeon2(constants.kGyroId)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.gyro.set_yaw(-50)

        self.odometry = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.getPigeonRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            Pose2d(),
        )

        self.field = Field2d()

        SmartDashboard.putData("odo_raw", self.field)

        self.nt = ntcore.NetworkTableInstance.getDefault()
        sms_topic = self.nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.sms_pub = sms_topic.publish()

        smst_topic = self.nt.getStructArrayTopic("/SwerveStatesTarget", SwerveModuleState)
        self.smst_pub = smst_topic.publish()

        self.limelight_tables = ["limelight-three","limelight-gee"]

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, _: self.driveRobotRelative(
                speeds
            ),  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0),  # Rotation PID constants
            ),
            RobotConfig.fromGUISettings(),
            lambda: DriverStation.getAlliance()
            == DriverStation.Alliance.kRed,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed.m_as("meter / second"), ySpeed.m_as("meter / second"), rot.m_as("radian / second"), self.getPigeonRotation2d()  # type: ignore
            )
        )

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, constants.kMaxSpeed.m_as("meter / second")
        )

        target_states = [
            self.frontLeft.setDesiredState(swerveModuleStates[0]),
            self.frontRight.setDesiredState(swerveModuleStates[1]),
            self.backLeft.setDesiredState(swerveModuleStates[2]),
            self.backRight.setDesiredState(swerveModuleStates[3]),
        ]

        self.smst_pub.set(target_states)

    def driveRobotRelative(self, robotRelativeSpeeds: ChassisSpeeds):
        # TODO: Discretization
        swerveModuleStates = self.kinematics.toSwerveModuleStates(robotRelativeSpeeds)

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, constants.kMaxSpeed.m_as("meter / second")
        )

        target_states = [
            self.frontLeft.setDesiredState(swerveModuleStates[0]),
            self.frontRight.setDesiredState(swerveModuleStates[1]),
            self.backLeft.setDesiredState(swerveModuleStates[2]),
            self.backRight.setDesiredState(swerveModuleStates[3]),
        ]

        self.smst_pub.set(target_states)

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""

        self.frontLeft.update()
        self.frontRight.update()
        self.backLeft.update()
        self.backRight.update()

        rotation2d = self.getPigeonRotation2d()
        self.odometry.update(
            rotation2d,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.update_nt_orientation(rotation2d)
        self.insert_limelight_measurement()
        SmartDashboard.putNumber("x", self.odometry.getEstimatedPosition().x_feet)
        SmartDashboard.putNumber("y", self.odometry.getEstimatedPosition().y_feet)
        SmartDashboard.putNumber("yaw", (rotation2d.radians()))

        self.field.setRobotPose(self.getPose())
        self.sms_pub.set(
            [
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            ]
        )

    def getPigeonRotation2d(self) -> Rotation2d:
        if self._isReal:
            return Rotation2d.fromDegrees(self.gyro.get_yaw().refresh().value)
        else:
            chSpds = self.kinematics.toChassisSpeeds(
                (
                    self.frontLeft.getState(),
                    self.frontRight.getState(),
                    self.backLeft.getState(),
                    self.backRight.getState(),
                )
            )
            self._simPose = self._simPose.exp(
                Twist2d(chSpds.vx * 0.02, chSpds.vy * 0.02, chSpds.omega * 0.02)
            )
            noise = Rotation2d.fromDegrees(random.uniform(-1.25, 1.25))
            return self._simPose.rotation() + noise

    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def resetPose(self, pose: Pose2d) -> None:
        self.odometry.resetPosition(
            self.getPigeonRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(
            (
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            )
        )

    def periodic(self) -> None:
        self.updateOdometry()
