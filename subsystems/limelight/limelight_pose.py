from dataclasses import dataclass
from ntcore import NetworkTableEntry
from wpimath.geometry import Rotation2d, Pose2d
from wpimath import units

@dataclass(init=False, eq=True)
class LimelightPose:
    """Class for keeping track of a limelight pose from a nttables entry with attributes."""

    entry: NetworkTableEntry
    # TODO: annotate with wpimath.units instead of floats
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    latency: float
    tag_count: float
    tag_span: float
    avg_tag_distance: float
    avg_tag_area: float
    update_time: int
    stddev_x: float
    stddev_y: float
    stddev_yaw: float
    invalid: bool

    def __init__(self, entry: NetworkTableEntry, stddev_entry: NetworkTableEntry):
        ts_value = entry.getAtomic()
        pose = ts_value.value
        stddevs = stddev_entry.getDoubleArray(None)

        if pose is None or stddevs is None:
            self.invalid = True
            return

        self.x = pose[0]
        self.y = pose[1]
        self.z = pose[2]
        self.roll = pose[3]
        self.pitch = pose[4]
        self.yaw = pose[5]
        self.latency = pose[6]
        self.timestamp = ts_value.time
        self.tag_count = pose[7]
        self.tag_span = pose[8]
        self.avg_tag_distance = pose[9]
        self.avg_tag_area = pose[10]
        self.stddev_x = stddevs[6]
        self.stddev_y = stddevs[7]
        self.stddev_yaw = stddevs[11]
        self.invalid = self.x == self.y == self.z or self.tag_count <= 0

    def time(self):
        """
        Gets the time the limelight pose was measured.
        :return: time in milliseconds epoch is FPGA
        """""
        return (self.timestamp / 1000000.0) - (self.latency / 1000.0)

    def covariance(self) -> tuple[float, float, float]:
        return self.stddev_x, self.stddev_y, self.stddev_yaw

    def as_pose(self) -> Pose2d:
        return Pose2d(self.x, self.y, Rotation2d(units.degreesToRadians(self.yaw)))
