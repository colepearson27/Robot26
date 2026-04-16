package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav;
    Pose3d robotPose = new Pose3d();

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

            RobotContainer.drivebase.addQuestPose(robotPose.toPose2d(), timestamp);

            String robotPoseString = robotPose.toString();
            SmartDashboard.putString(Constants.SmartDashboardKeys.QUEST_POSE, robotPoseString);

            SmartDashboard.putNumber("Quest Battery Percentage", questNav.getBatteryPercent().getAsInt());
        }
    }

    public void resetQuestPose(Pose3d newPose) {
        questNav.setPose(newPose);
    }

    public Pose2d getQuestPose() {
        return robotPose.toPose2d();
    }
}
