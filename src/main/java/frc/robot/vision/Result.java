package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class Result {

    public final double timestamp;
    public final Pose3d pose;

    public Result(double timestamp, Pose3d pose) {
        this.timestamp = timestamp;
        this.pose = pose;
    }
}
