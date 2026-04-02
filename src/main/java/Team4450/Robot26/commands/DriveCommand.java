package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
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
        Pose2d drivebasePose = drivebase.getODPose();
        if (!drivebase.wallTrackingLeft && !drivebase.wallTrackingRight) {
            targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
        } else {
            if (drivebase.wallTrackingLeft) {
                targetHeading = -25;
            } else if (drivebase.wallTrackingRight) {
                targetHeading = 25;
            } else if (drivebase.ferryTracking) {
                targetHeading = 180;
            } else {
                targetHeading = drivebase.getAngleToAim(drivebasePose, hubPosition);
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
            } else {
                RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
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
