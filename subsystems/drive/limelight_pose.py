from dataclasses import dataclass
from ntcore import NetworkTableEntry


@dataclass(init=False, eq=True)
class LimelightPose:
    """Class for keeping track of a limelight pose from a nttables entry with attributes."""

    entry: NetworkTableEntry
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

    def __init__(self, entry: NetworkTableEntry, stddev_entry: NetworkTableEntry):
        pose = entry.getDoubleArray(None)
        if pose is None:
            raise AttributeError("Double Array not found as entry value")
        stddevs = stddev_entry.getDoubleArray(None)
        if stddevs is None:
            raise AttributeError("TODO: fill in")
        self.x = pose[0]
        self.y = pose[1]
        self.z = pose[2]
        self.roll = pose[3]
        self.pitch = pose[4]
        self.yaw = pose[5]
        self.latency = pose[6]
        self.tag_count = pose[7]
        self.tag_span = pose[8]
        self.avg_tag_distance = pose[9]
        self.avg_tag_area = pose[10]
        self.update_time = entry.getLastChange()
        self.stddev_x = stddevs[6]
        self.stddev_y = stddevs[7]
        self.stddev_yaw = stddevs[11]

    def time(self):
        # TODO: Units
        # TODO: is timestamp the right timestamp
        return self.update_time - self.latency

    def covariance(self):
        return (self.stddev_x, self.stddev_y, self.stddev_yaw)
