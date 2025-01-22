import commands2
import ntcore
from phoenix6.hardware import Pigeon2
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import SmartDashboard

from constants import Limelight as constants
from subsystems.limelight.limelight_pose import LimelightPose
from subsystems.drivetrain import CommandSwerveDrivetrain as Drivetrain

class Limelight(commands2.Subsystem):
    def __init__(self, drive: Drivetrain):
        super().__init__()
        self.limelight_tables = ["limelight-three", "limelight-gee"]
        self.gyro = Pigeon2(constants.kGyroId)
        self.drivetrain = drive
        self.nt = ntcore.NetworkTableInstance.getDefault()

    def update_nt_orientation(self) -> None:
        """Updates the network tables of every limelight with robot orientation data from the IMU"""
        # TODO: doesn't update velocities (see angular velocity stuff)
        # SET Robot Orientation and angular velocities in degrees and degrees per second[yaw,yawrate,pitch,pitchrate,roll,rollrate]
        rotation_list = [self.gyro.get_yaw().value_as_double%360, 0.0, 0.0, 0.0, 0.0, 0.0]
        for table in self.limelight_tables:
            entry = self.nt.getTable(table).getEntry("robot_orientation_set")
            # Time of 0 is equivalent to the current instant
            entry.setDoubleArray(rotation_list, 0)

    def insert_limelight_measurements(self) -> None:
        """Gets all poses from the limelights and gives them to the odometry"""
        poses: list[LimelightPose] = []
        # Get all poses from limelights
        for table in self.limelight_tables:
            entry = self.nt.getTable(table).getEntry("botpose_orb_wpired")
            stddev_entry = self.nt.getTable(table).getEntry("stddevs")
            try:
                pose = LimelightPose(entry, stddev_entry)
                poses.append(pose)
            except AttributeError:
                pass

        for pose in poses:
            if pose.tag_count > 0:
                pose2d = Pose2d(pose.x, pose.y, Rotation2d(pose.yaw))
                self.drivetrain.add_vision_measurement(
                    pose2d,
                    # .time() returns milliseconds but .addVisionMeasurement requires seconds
                    # Epochs are both FPGA, no conversion needed
                    units.millisecondsToSeconds(pose.time()),
                    # pose covariance is in meters
                    pose.covariance()
                )

    def periodic(self) -> None:
        self.update_nt_orientation()
        self.insert_limelight_measurements()
        SmartDashboard.putNumber("gyro", self.gyro.get_yaw().value%360)