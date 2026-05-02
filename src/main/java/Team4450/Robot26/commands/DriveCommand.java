package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import Team4450.Lib.Util;
import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.subsystems.Drivebase;
import static Team4450.Robot26.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command {
    private final Drivebase drivebase;

    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationXSupplier;
    private final DoubleSupplier rotationYSupplier;
    public final PIDController headingPID;

    public DriveCommand(Drivebase driveBase,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier rotationXSupplier,
            DoubleSupplier rotationYSupplier,
            PIDController headingPID) {
        Util.consoleLog();

        this.drivebase = driveBase;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationXSupplier = rotationXSupplier;
        this.rotationYSupplier = rotationYSupplier;
        this.headingPID = headingPID;
        headingPID.setIntegratorRange(-ROBOT_HEADING_KI_MAX, ROBOT_HEADING_KI_MAX);
        headingPID.enableContinuousInput(-180, 180);
        headingPID.setTolerance(ROBOT_HEADING_TOLERANCE_DEG);

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        // This is the default command for the Drivebase. When running in autonmous, the
        // auto commands
        // require Drivebase, which preempts the default Drivebase command. However, if
        // our auto code ends
        // before end of auto period, then this drive command resumes and is feeding
        // drivebase during remainder
        // of auto period. This was not an issue until the joystick drift problems
        // arose, so the resumption of a
        // driving command during auto had the robot driving randomly after our auto
        // program completed. The if
        // statment below prevents this.

        if (robot.isAutonomous()) return; // We do not want to run the drive command if we are in auto

        // Somewhere in here for the pose estimate for the robot there is a problem where on the red side the robot seems to point to the left. There does not seem to be a problem with the blue side

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
        Pose2d drivebasePose = drivebase.getPose();

        if (!drivebase.wallTrackingLeft && !drivebase.wallTrackingRight) {
            if (Math.abs(rotationXSupplier.getAsDouble()) <= 0.2 && Math.abs(rotationYSupplier.getAsDouble()) <= 0.2 && alliance == DriverStation.Alliance.Blue) {
                // Checks if robot is currently in the Alliance Zone then aims at the hub
                if (drivebasePose.getX() < NEUTRAL_BLUE_ZONE_BARRIER_X) {
                    targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
                    RobotContainer.shooter.disableManualDistanceThree();
                } else {
                    // Checks what side the robot is on, and aims at the nearest ferrying target point predefined in Constants
                    if (drivebasePose.getY() < FIELD_MIDDLE_Y) {
                        RobotContainer.shooter.enableManualDistanceThree();
                        RobotContainer.shooter.disableManualDistanceOne();
                        RobotContainer.shooter.disableManualDistanceTwo();
                        targetHeading = drivebase.getAngleToAim(drivebasePose, FERRY_BLUE_OUTPOST_CORNER);
                    } else {
                        RobotContainer.shooter.enableManualDistanceThree();
                        RobotContainer.shooter.disableManualDistanceOne();
                        RobotContainer.shooter.disableManualDistanceTwo();
                        targetHeading = drivebase.getAngleToAim(drivebasePose, FERRY_BLUE_BLANK_CORNER);
                    }
                }
                // This does the same thing but for the red alliance
            } else if (Math.abs(rotationXSupplier.getAsDouble()) <= 0.2 && Math.abs(rotationYSupplier.getAsDouble()) <= 0.2 && alliance == DriverStation.Alliance.Red) {
                if (drivebasePose.getX() > NEUTRAL_RED_ZONE_BARRIER_X) {
                    targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
                    RobotContainer.shooter.disableManualDistanceThree();
                } else {
                    if (drivebasePose.getY() < FIELD_MIDDLE_Y) {
                        RobotContainer.shooter.enableManualDistanceThree();
                        RobotContainer.shooter.disableManualDistanceOne();
                        RobotContainer.shooter.disableManualDistanceTwo();
                        targetHeading = drivebase.getAngleToAim(drivebasePose, FERRY_RED_BLANK_CORNER);
                    } else {
                        RobotContainer.shooter.enableManualDistanceThree();
                        RobotContainer.shooter.disableManualDistanceOne();
                        RobotContainer.shooter.disableManualDistanceTwo();
                        targetHeading = drivebase.getAngleToAim(drivebasePose, FERRY_RED_OUTPOST_CORNER);
                    }
                }
            } else {
                targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
                RobotContainer.shooter.disableManualDistanceThree();
            }
        } else {
            if (drivebase.wallTrackingLeft) {
                targetHeading = -25;
            } else if (drivebase.wallTrackingRight) {
                targetHeading = 25;
            } else {
                targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
                // targetHeading = -Math.toDegrees(Math.atan2(rotationYSupplier.getAsDouble(), rotationXSupplier.getAsDouble())) - 90;
            }
        }

        targetHeading = normalizeAngle(targetHeading);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.TARGET_HEADING, targetHeading);

        double drivebaseYaw = drivebase.getODPose().getRotation().getDegrees();
        double headingError = drivebaseYaw - targetHeading;
        SmartDashboard.putNumber("Heading Error", headingError);

        if (Constants.HUB_TRACKING) {
            // Uses a PID and the previous assigned target heading to rotate there
            double rotation = headingPID.calculate(drivebaseYaw, targetHeading);
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.HEADING_PID_OUTPUT, rotation);
            double throttle = throttleSupplier.getAsDouble();
            double strafe = strafeSupplier.getAsDouble();

            throttle = Util.squareInput(throttle);
            strafe = Util.squareInput(strafe);

            headingPID.setP(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_P, Constants.ROBOT_HEADING_KP));
            headingPID.setI(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_I, Constants.ROBOT_HEADING_KI));
            headingPID.setD(SmartDashboard.getNumber(Constants.SmartDashboardKeys.HEADING_D, Constants.ROBOT_HEADING_KD));

            if (Math.abs(headingError) <= 5 || Math.abs(headingError) >= 355) {
                RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.3);
                rotation = 0;
            } else {
                RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
            }

            Pose2d goalPose = (RobotContainer.shooter.getGoalPose());
            double xDiff = Math.abs(goalPose.getX() - drivebase.getPose().getX());
            double yDiff = Math.abs(goalPose.getY() - drivebase.getPose().getY());
            double distToGoal = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

            if (distToGoal < 2 && Math.abs(headingError) < 7) {
                rotation = 0;
                drivebase.setX();
            } else if (distToGoal > 2 && distToGoal < 4 && Math.abs(headingError) < 5) {
                rotation = 0;
                drivebase.setX();
            } else if (distToGoal > 4 && Math.abs(headingError) < 3) {
                rotation = 0;
                drivebase.setX();
            }

            drivebase.drive(throttle, strafe, rotation);
            return;
        } else {
            RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
            double rotation = rotationXSupplier.getAsDouble();
            if (rotation <= 0.1) {
                rotation = rotation * 2;
            }
            double throttle = throttleSupplier.getAsDouble();
            double strafe = strafeSupplier.getAsDouble();

            // Squaring input is one way to ramp JS inputs to reduce sensitivity.
            // Please do not square the headingPID

            throttle = Util.squareInput(throttle);
            strafe = Util.squareInput(strafe);
            // rotation = Util.squareInput(rotation);
            // rotation = Math.pow(rotation, 5);

            drivebase.drive(throttle, strafe, rotation);
            return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }

    public double normalizeAngle(double angle) {
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [-360, 360).
        double normalizedAngle = angle % 360;

        // If the result was negative, shift it to the range [0, 360).
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }

        // If the angle is in the range [180, 360), shift it to [-180, 0).
        if (normalizedAngle >= 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }
}
