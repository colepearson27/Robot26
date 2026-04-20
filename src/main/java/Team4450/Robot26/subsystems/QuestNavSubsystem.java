package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.robot;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.Constants.SmartDashboardKeys;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav;
    Pose3d robotPose = new Pose3d();
    double robotRotation;

    public QuestNavSubsystem() {
        questNav = new QuestNav(); 

        questNav.setVersionCheckEnabled(false);

        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_CONNECTED, false);
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_TRACKING, false);
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_LOW_BATTERY, false);
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.USE_QUEST, true);

        questNav.onConnected(() -> SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_CONNECTED, true));
        questNav.onDisconnected(() -> SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_CONNECTED, false));

        questNav.onTrackingAcquired(() -> SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_TRACKING, true));
        questNav.onTrackingLost(() -> SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_TRACKING, false));
        questNav.onLowBattery(20, level -> SmartDashboard.putBoolean(Constants.SmartDashboardKeys.QUEST_LOW_BATTERY, true));

        // Initialize a blank pose3d
        questNav.setPose(new Pose3d());
    }
    @Override
    public void periodic() {
        questNav.commandPeriodic();

        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            //get quest pose
            Pose3d questPose = questFrame.questPose3d();
            //get timestamp for that pose
            double timestamp = questFrame.dataTimestamp();

            robotPose = questPose.transformBy(Constants.ROBOT_TO_QUEST.inverse());
            robotRotation = robotPose.getRotation().toRotation2d().getDegrees();

            RobotContainer.drivebase.addQuestPose(robotPose.toPose2d(), timestamp);

            String robotPoseString = robotPose.toString();

            SmartDashboard.putString(Constants.SmartDashboardKeys.QUEST_POSE, robotPoseString);
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.QUEST_BATTERY_PERCENTAGE, questNav.getBatteryPercent().getAsInt());
        }
    }

    public void resetQuestPose(Pose3d newPose) {
        questNav.setPose(newPose);
    }

    public void resetQuest2d(Pose2d newPose) {
        Pose3d pose = new Pose3d(newPose);

        questNav.setPose(pose);
    }

    public Pose2d getQuestPose() {
        return robotPose.toPose2d();
    }

    public boolean useQuest() {
        if (SmartDashboard.getBoolean(Constants.SmartDashboardKeys.USE_QUEST, true) &&
                SmartDashboard.getBoolean(Constants.SmartDashboardKeys.QUEST_TRACKING, false) &&
                !questAgainstWall()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean questAgainstWall() {
        if ((robotPose.getX() < 0.65 && robotRotation > 172 && robotRotation < -172) 
                || (robotPose.getX() > 16 && robotRotation < 8 && robotRotation > -8)
                || (robotPose.getY() < 0.6 && robotRotation > -98 && robotRotation < -82)
                || (robotPose.getY() > 7.5 && robotRotation < 98 && robotRotation > 82)) {
            return true;
        } else {
            return false;
        }
    }
}
