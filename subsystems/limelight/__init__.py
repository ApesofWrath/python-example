# TODO: https://github.com/SteelRidgeRobotics/2025Reefscape/blob/21dd224745dc5b56c75622affa622bdc86a033e0/subsystems/swerve.py#L359

import commands2
import ntcore
from phoenix6 import utils
from phoenix6.hardware import Pigeon2
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import SmartDashboard, Field2d
from wpilib import RobotState

from constants import Limelight as constants
from subsystems.limelight.limelight_pose import LimelightPose
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

from pipe import Pipe, map, filter
from limelight import LimelightHelpers

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.pigeon2 = Pigeon2(constants.kGyroId)
        self.pigeon2.set_yaw(0)
        self.seedingDone = False

        self.fields = {name: Field2d() for name in constants.kLimelightHostnames}
        for name in self.fields.keys():
            SmartDashboard.putData("limelight_" + name, self.fields[name])

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

            self.fields[LLHostname].setRobotPose(mega_tag2.pose)
        else:
            self.fields[LLHostname].setRobotPose(Pose2d(-1,-1,Rotation2d(units.degreesToRadians(180))))

    def periodic(self) -> None:
        for llhn in constants.kLimelightHostnames:
            self.insert_limelight_measurements(llhn)
        SmartDashboard.putNumber("gyro", self.pigeon2.get_yaw().value%360)

class OldLimelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.drivetrain = drive
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.gyro = Pigeon2(constants.kGyroId)
        self.gyro.set_yaw(0)
        self.fields = {name: Field2d() for name in constants.kLimelightHostnames}
        for name in self.fields.keys():
            SmartDashboard.putData("limelight_" + name, self.fields[name])
        self.seedingDone = False

    def update_nt_orientation(self) -> None:
        """Updates the network tables of every limelight with robot orientation data from the IMU"""
        for table in constants.kLimelightHostnames:
            nttable = self.nt.getTable(table)
            if RobotState.isDisabled():
                nttable.getEntry("imumode_set").setDouble(1)
                # TODO: doesn't update velocities (see angular velocity stuff)
                # SET Robot Orientation and angular velocities in degrees and degrees per second[yaw,yawrate,pitch,pitchrate,roll,rollrate]
                rotation_list = [self.gyro.get_yaw().value_as_double%360, 0.0, 0.0, 0.0, 0.0, 0.0]
                nttable.getEntry("robot_orientation_set").setDoubleArray(rotation_list, 0) # Time of 0 is equivalent to the current instant
            elif RobotState.isEnabled():
                self.seedingDone = True
                nttable.getEntry("imumode_set").setDouble(2)

    @Pipe
    def runOn(iterable, funct):
        """Like map but not lazy and also return the origional value"""
        for item in iterable:
            funct(item)
            yield item

    def insert_limelight_measurements(self) -> None:
        # Use pipe library to chain maps together without nasty arg nesting
        list( # pipelines are lazy, we need to acess the value to make it do the pipeline
            constants.kLimelightHostnames | # the limelight hostnames
            map(lambda hostname: (self.nt.getTable(hostname), hostname)) | # tuple of the hostname and nttables table
            map(lambda table:( # tuple of hostname and LLP
                LimelightPose(
                    table[0].getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([]),
                    table[0].getEntry("stddevs")
                ),
                table[1]
            )) |
            self.runOn(lambda pose: self.fields[pose[1]].setRobotPose(
                pose[0].as_pose()
                if not pose[0].invalid else
                Pose2d(-1,-1,Rotation2d(units.degreesToRadians(180)))
            )) | # update all the individual limelight fields
            map(lambda pose_tuple: pose_tuple[0]) | # drop the hostname, it was just for updating the fields dict
            filter(lambda pose: not pose.invalid and self.gyro.get_angular_velocity_z_world().value < 80) | # drop invalid poses
            # TODO: use pose covariance as a filter even with hardcoded covars
            self.runOn(lambda pose:
                self.drivetrain.add_vision_measurement(
                    Pose2d(pose.x, pose.y, Rotation2d(units.degreesToRadians(pose.yaw))),
                    # .time() returns milliseconds but .addVisionMeasurement requires seconds
                    # Epochs are both FPGA, no conversion needed
                    pose.time(),
                    # pose covariance is in meters
                    pose.covariance()
                    #(.005,.005,.001)
                )
            ) |
            self.runOn(lambda pose:
                SmartDashboard.putNumberArray("covaiance", pose.covariance())
            )
        )

    def periodic(self) -> None:
        if not self.seedingDone:
            self.update_nt_orientation()	
        self.insert_limelight_measurements()
        SmartDashboard.putNumber("gyro", self.gyro.get_yaw().value%360)