package Team4450.Robot26.utility;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPose {
    public Pose2d pose;
    public long timestamp;
    public VisionPose nextNode;

    public VisionPose(Pose2d pose, long timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
