package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import Team4450.Lib.Util;
import Team4450.Robot26.Constants;
import Team4450.Robot26.subsystems.Drivebase;
import static Team4450.Robot26.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnableHubTracking extends Command {

  public final PIDController headingPID;

  Drivebase drivebase;

  public EnableHubTracking(Drivebase driveBase, PIDController headingPID) {

    Util.consoleLog();

    this.drivebase = driveBase;
    this.headingPID = headingPID;

    headingPID.setIntegratorRange(-ROBOT_HEADING_KI_MAX, ROBOT_HEADING_KI_MAX);
    headingPID.enableContinuousInput(-180, 180);
    headingPID.setTolerance(ROBOT_HEADING_TOLERANCE_DEG);

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    drivebase.enableHubTracking();
  }

  @Override
  public void execute() {
    if (Constants.HUB_TRACKING) {

      // This finds where the correct hub position is
      Pose2d hubPosition;
      if (alliance == DriverStation.Alliance.Blue) {
        hubPosition = new Pose2d(HUB_BLUE_WELDED_POSE.getX(), HUB_BLUE_WELDED_POSE.getY(), Rotation2d.kZero);
      } else {
        hubPosition = new Pose2d(HUB_RED_WELDED_POSE.getX(), HUB_RED_WELDED_POSE.getY(), Rotation2d.kZero);
      }

      double targetHeading;

      // Decides where to track
      // If both inputs are zero and the alliance is blue then
      if (alliance == DriverStation.Alliance.Blue) {
        // Checks if robot is currently in the Alliance Zone then aims at the hub
        if (drivebase.getPose().getX() < NEUTRAL_BLUE_ZONE_BARRIER_X) {
          targetHeading = drivebase.getAngleToAim((hubPosition));
        } else {
          // Checks what side the robot is on, and aims at the nearest ferrying target
          // point predefined in Constants
          if (drivebase.getPose().getY() < FIELD_MIDDLE_Y) {
            targetHeading = drivebase.getAngleToAim((FERRY_BLUE_OUTPOST_CORNER));
          } else {
            targetHeading = drivebase.getAngleToAim((FERRY_BLUE_BLANK_CORNER));
          }
        }
        // This does the same thing but for the red alliance
      } else if (alliance == DriverStation.Alliance.Red) {
        if (drivebase.getPose().getX() > NEUTRAL_RED_ZONE_BARRIER_X) {
          targetHeading = drivebase.getAngleToAim(hubPosition);
        } else {
          if (drivebase.getPose().getY() < FIELD_MIDDLE_Y) {
            targetHeading = drivebase.getAngleToAim((FERRY_RED_BLANK_CORNER));
          } else {
            targetHeading = drivebase.getAngleToAim((FERRY_RED_OUTPOST_CORNER));
          }
        }
        // If there IS input, set the target heading to where the joystick si facing in
        // relation to the driver
      } else {
        targetHeading = 0;
      }

      SmartDashboard.putNumber(Constants.SmartDashboardKeys.TARGET_HEADING, targetHeading);

      double error = -targetHeading + Math.toDegrees(drivebase.getPose().getRotation().getDegrees());

      SmartDashboard.putNumber(Constants.SmartDashboardKeys.HEADING_ERROR, error);

      // Uses a PID and the previous assigned target heading to rotate there
      double rotation = -headingPID.calculate(-Math.toDegrees(drivebase.getPose().getRotation().getDegrees()),
          -targetHeading);

      headingPID.setP(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_P, Constants.ROBOT_HEADING_KP));
      headingPID.setI(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_I, Constants.ROBOT_HEADING_KI));
      headingPID.setD(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_D, Constants.ROBOT_HEADING_KD));

      drivebase.drive(0, 0, rotation);

      return;
    }
  }

  @Override
  public boolean isFinished() {
    return false;

  }

  @Override
  public void end(boolean inturrupted) {

  }
}
