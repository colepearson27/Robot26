package Team4450.Robot26.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class VisionBuffer {
    public VisionPose lastNode;

    public VisionBuffer() {
    }

    public VisionPose getLatest() {
        return lastNode;
    }

    public VisionPose get(int targetIndex) {
        if (targetIndex == 0) {
            return lastNode;
        }
        int i = 0;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            if (i == targetIndex) {
                return node;
            }
            node = node.nextNode;
            i++;
        }
        return null;
    }

    public int length() {
        int i = 1;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            node = node.nextNode;
            i++;
        }
        return i;
    }

    public void append(VisionPose poseToAdd) {
        if (lastNode == null) {
            lastNode = poseToAdd;
            return;
        }
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            node = node.nextNode;
        }
        node.nextNode = poseToAdd;
    }

    public void remove(VisionPose elementBefore, VisionPose poseToRemove) {
        if (poseToRemove.nextNode == null) {
            elementBefore.nextNode = null;
        } else {
            elementBefore.nextNode = poseToRemove.nextNode;
        }
    }

    public void truncate(int len) {
        if (len == 0) {
            lastNode = null;
            return;
        }
        int i = 1;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            if (i > len) {
                node.nextNode = null;
                return;
            }
            node = node.nextNode;
            i++;
        }
    }

    public void replayMotion() {
        if (lastNode == null) {
            return;
        }

        VisionPose node = lastNode;
        Pose2d poseEstimate = node.pose;
        long timestampSeconds = node.timestamp;
        while (node.nextNode != null) {
            Transform2d motion = poseEstimate.minus(node.nextNode.pose);
            long timestampSecondsDifference = node.nextNode.timestamp = timestampSeconds;
            // TODO: If motion impossible
            node = node.nextNode;
            poseEstimate = node.pose;
        }
    }

    // public Pose2d getPoseEstimate() {
    // }
}
