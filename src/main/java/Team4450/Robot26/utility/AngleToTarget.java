package Team4450.Robot26.utility;

import Team4450.Robot26.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AngleToTarget {
    // All targeting should be based on the WPIBlue coordinate system
    public double getAngleToFaceGoalDegrees(Pose2d robotPosition) {
        // If blue side
        double xDiff = 0;
        double yDiff = 0;
        if (Constants.alliance == Alliance.Blue) {
            xDiff = Constants.HUB_BLUE_X - robotPosition.getX();
            yDiff = Constants.HUB_BLUE_Y + robotPosition.getY();
            // If red side
        } else if (Constants.alliance == Alliance.Red) {
            xDiff = Constants.HUB_RED_X + robotPosition.getX();
            yDiff = Constants.HUB_RED_Y - robotPosition.getY();
        } else {
            // Error
        }
        // Return degrees to angle to face the target
        // This code does not take into account the curret robot angle because we want to set this as the target robot angle
        return (Math.toDegrees(Math.atan(yDiff / xDiff)));
    }
}
