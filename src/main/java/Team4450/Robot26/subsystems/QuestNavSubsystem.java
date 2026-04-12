package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import Team4450.Lib.Util;
import Team4450.Robot26.utility.ConsoleEveryX;
import edu.wpi.first.math.geometry.Pose2d;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav = new QuestNav(); 

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            //get quest pose
            Pose3d questPose = questFrame.questPose3d();
            //get timestamp for that pose
            double timestamp = questFrame.dataTimestamp();

            Pose3d robotPose = questPose.transformBy(Constants.ROBOT_TO_QUEST.inverse());

            RobotContainer.drivebase.addQuestPose(robotPose.toPose2d(), timestamp);

            String robotPoseString = robotPose.toString();
            SmartDashboard.putString(Constants.SmartDashboardKeys.QUEST_POSE, robotPoseString);
        }
    }

    public void resetQuestPose(Pose3d newPose) {
        questNav.setPose(newPose);
    }
}
