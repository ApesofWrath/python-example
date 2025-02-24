import commands2, math
from phoenix6 import utils, swerve
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, Field2d
from pathplannerlib.path import PathConstraints
from pathplannerlib.auto import AutoBuilder
from wpilib import SmartDashboard, Field2d, DriverStation
from wpimath.geometry import Pose2d, Transform2d
from wpimath.units import degreesToRadians
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

import constants
from subsystems.limelight.limelight import LimelightHelpers
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.Limelight.kGyroId)
        self.pigeon2.set_yaw((DriverStation.getAlliance() == DriverStation.Alliance.kRed) * 180)
        self.getDelta()

        for id,target in constants.Limelight.kAlignmentTargets.items():
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData("alignTarget "+str(id), field)

	# thank you Steel Ridge/team 6343
    def insert_limelight_measurements(self, LLHostname: str) -> None:
        """
        Add vision measurement to MegaTag2
        """

        LimelightHelpers.set_robot_orientation(
            LLHostname,
            self.pigeon2.get_yaw().value,
            self.pigeon2.get_angular_velocity_z_world().value,
            self.pigeon2.get_pitch().value,
            self.pigeon2.get_angular_velocity_y_world().value,
            self.pigeon2.get_roll().value,
            self.pigeon2.get_angular_velocity_x_world().value
        )

        # get botpose estimate with origin on blue side of field
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(LLHostname)
        
        # if we are spinning slower than 720 deg/sec and we see tags
        if abs(self.pigeon2.get_angular_velocity_z_world().value) <= 720 and mega_tag2.tag_count > 0:
            # set and add vision measurement
            self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 0.1)) #(0.7, 0.7, 9999999)
            self.drivetrain.add_vision_measurement(mega_tag2.pose, utils.fpga_to_current_time(mega_tag2.timestamp_seconds))

    def pathfind(self) -> commands2.Command:
        return AutoBuilder.pathfindToPose(
            constants.Limelight.kAlignmentTargets[9],
            PathConstraints( 2.5, 2.5, 1, 1 )
		)

    def getDelta(self) -> int:
        current = self.drivetrain.get_state().pose
        self.delta = current.log(constants.Limelight.kAlignmentTargets[9])

    def align(self) -> commands2.Command:
        return commands2.RepeatCommand(
            commands2.RunCommand(
                lambda: self.drivetrain.set_control(
                    swerve.requests.FieldCentric() \
                        .with_rotational_rate(self.delta.dtheta * constants.Limelight.precise.spin_p * (abs(self.delta.dtheta_degrees) > constants.Limelight.precise.theta_tolerance)) \
                        .with_velocity_y(-self.delta.dy * constants.Limelight.precise.move_p * (abs(self.delta.dy) > constants.Limelight.precise.xy_tolerance)) \
                        .with_velocity_x(-self.delta.dx * constants.Limelight.precise.move_p * (abs(self.delta.dx) > constants.Limelight.precise.xy_tolerance))
                )
            )
        )

    def periodic(self) -> None:
        for llhn in constants.Limelight.kLimelightHostnames:
            self.insert_limelight_measurements(llhn)
        self.getDelta()
        SmartDashboard.putNumber("gyro", self.pigeon2.get_yaw().value%360)
        SmartDashboard.putString("alignDelta", f"({self.delta.dx:.10f},{self.delta.dy:.10f}) {self.delta.dtheta_degrees:.10f}")