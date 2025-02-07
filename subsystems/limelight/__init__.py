import commands2
from phoenix6 import utils
from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, Field2d
from pathplannerlib.path import PathConstraints
from pathplannerlib.auto import AutoBuilder

import constants
from constants import makeCommand
from subsystems.limelight.limelight import LimelightHelpers
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.Limelight.kGyroId)
        self.pigeon2.set_yaw(0)

        for target in constants.Limelight.kAlignmentTargets:
            field = Field2d()
            field.setRobotPose(target)
            SmartDashboard.putData(f"alignTarget ({target.X()},{target.Y()})", field)

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
            self.drivetrain.set_vision_measurement_std_devs((0.7, 0.7, 9999999))
            self.drivetrain.add_vision_measurement(mega_tag2.pose, utils.fpga_to_current_time(mega_tag2.timestamp_seconds))

    def align(self) -> commands2.Command:
        # TODO: figure out pathplanner team
        # TODO: it goes in the exact opposite direction
        return AutoBuilder.pathfindToPose(
            self.drivetrain.get_state().pose.nearest(constants.Limelight.kAlignmentTargets),
            PathConstraints( 3, 3, 540, 720 )
		)

    def periodic(self) -> None:
        for llhn in constants.Limelight.kLimelightHostnames:
            self.insert_limelight_measurements(llhn)
        SmartDashboard.putNumber("gyro", self.pigeon2.get_yaw().value%360)