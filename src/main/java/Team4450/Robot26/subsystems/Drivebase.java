package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.DriveConstants.*;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import Team4450.Lib.Util;
import static Team4450.Robot26.Constants.*;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.Constants.DriveConstants;
import Team4450.Robot26.commands.DriveCommand;
import Team4450.Robot26.subsystems.SDS.CommandSwerveDrivetrain;
import Team4450.Robot26.subsystems.SDS.Telemetry;
import Team4450.Robot26.subsystems.SDS.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import Team4450.Robot26.utility.RobotOrientation;

/**
 * This class wraps the SDS drive base subsystem allowing us to add/modify drive
 * base
 * functions without modifyinig the SDS code generated from Tuner. Also allows
 * for
 * convenience wrappers for more complex functions in SDS code.
 */
public class Drivebase extends SubsystemBase {
  private CommandSwerveDrivetrain sdsDrivebase = TunerConstants.createDrivetrain();

  public PigeonWrapper pigeonWrapper = new PigeonWrapper(sdsDrivebase.getPigeon2());

  // This should init to whatever the limelights see during the init period,
  // otherwise set a smartdashboard and a console log into if it does not
  public Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kZero);
  public Pose2d limelightPoseEstimate = new Pose2d(0, 0, Rotation2d.kZero);

  private final Telemetry logger = new Telemetry(kMaxSpeed);

  // Field2d object creates the field display on the simulation and gives us an
  // API
  // to control what is displayed (the simulated robot).
  private final Field2d field2d = new Field2d();

  private boolean fieldRelativeDriving = true, slowMode = false;
  private boolean neutralModeBrake = true;
  private double maxSpeed = kMaxSpeed * kDriveReductionPct;
  private double maxRotRate = kMaxAngularRate * kRotationReductionPct;
  private boolean driverControlled = true;
  private boolean bumpHappened = true;

  private double lastThrottle = 0;
  private double lastStrafe = 0;
  private double lastRotation = 0;

  public boolean wallTrackingLeft = false;
  public boolean wallTrackingRight = false;

  private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
      .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
      .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
      .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public Drivebase() {
    Util.consoleLog();

        // Add pigeon gyro as a Sendable. Updates the dashboard heading indicator
        // automatically.
        SmartDashboard.putData(Constants.SmartDashboardKeys.PIGEON_GYRO, pigeonWrapper);
        SmartDashboard.putData(Constants.SmartDashboardKeys.FIELD2D, field2d);

    // Check Gyro.
    if (pigeonWrapper.getPigeon().isConnected())
      Util.consoleLog("Pigeon connected version=%s", pigeonWrapper.getPigeon().getVersion());
    else {
      Exception e = new Exception("Pigeon is NOT connected!");
      Util.logException(e);
    }

    // Set drive motors to brake when power is zero.
    sdsDrivebase.configNeutralMode(NeutralModeValue.Brake);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();

    RobotModeTriggers.disabled().whileTrue(
        sdsDrivebase.applyRequest(() -> idle).ignoringDisable(true));

        // Updates the dashboard heading indicator automatically.
        // Can I remove this?
        SmartDashboard.putData(Constants.SmartDashboardKeys.FIELD2D, field2d);

    // At some point move this to teleop init if it can be done quickly because
    // we will be waiting for the Limelight to get an accurate position during init
    // periodic
    resetOdometry(DriveConstants.DEFAULT_STARTING_POSE);

    // Under sim, we starting pose the robot (above) before you can change the
    // alliance
    // in the sim UI. We can't really do it anywhere else or it would interfere with
    // transition from auto to teleop. So we pose robot at lower left (blue) corner
    // and
    // force the blue driving perspective.

    if (RobotBase.isSimulation())
      driveField.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;

    // Register for SDS telemetry.
    // sdsDrivebase.registerTelemetry(logger::telemeterize);

    updateDS();
  }

  @Override
  public void periodic() {
    sdsDrivebase.periodic();

    // update 3d simulation: look in AdvantageScope.java for more
    // AdvantageScope.getInstance().setRobotPose(getPose());
    // AdvantageScope.getInstance().update();
    // AdvantageScope.getInstance().setSwerveModules(sdsDrivebase);

    // See this function for more information.
    updateModulePoses(sdsDrivebase);

    SmartDashboard.putString(Constants.SmartDashboardKeys.ROBOT_POSE, getPose().toString());
    SmartDashboard.putBoolean(Constants.SmartDashboardKeys.HUB_TRACKING, Constants.HUB_TRACKING);
    SmartDashboard.putBoolean(Constants.SmartDashboardKeys.BUMP_HAPPENED, this.bumpHappened);

    Pose2d drivebasePose = getPose();
    if (Math.abs(drivebasePose.getX()) > Constants.FIELD_MAX_X || Math.abs(drivebasePose.getY()) > Constants.FIELD_MAX_Y) {
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.OUTSIDE_FIELD, true);
    } else {
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.OUTSIDE_FIELD, false);
    }


    if (SmartDashboard.getNumber(Constants.SmartDashboardKeys.ROBOT_DISTANCE, 0) > 1.25 && SmartDashboard.getNumber(Constants.SmartDashboardKeys.ROBOT_DISTANCE, 0) < 1.75) {
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.DISTANCE_BOX, true);
    } else {
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.DISTANCE_BOX, false);
    }

    SmartDashboard.putNumber(Constants.SmartDashboardKeys.BATTERY_VOLTAGE, RobotController.getBatteryVoltage());

    if (RobotContainer.inTestMode) {
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.Gyro_HEADING, pigeonWrapper.startingYaw);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.GYRO_STARTING_YAW, pigeonWrapper.getHeading());
        SmartDashboard.putString(Constants.SmartDashboardKeys.ROBOT_OD_POSE, getODPose().toString());
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.DRIVEBASE_CURRENT, getDrivetrainCurrent());
        SmartDashboard.putString(Constants.SmartDashboardKeys.LIMELIGHT_POSE, this.limelightPoseEstimate.toString());
        SmartDashboard.putNumber("Flywheel Error", RobotContainer.shooter.flywheelRPMError);
        
    }
  }

  public double getDrivetrainCurrent() {
    return sdsDrivebase.getModule(0).getDriveMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(0).getSteerMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(1).getDriveMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(1).getSteerMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(2).getDriveMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(2).getSteerMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(3).getDriveMotor().getSupplyCurrent().getValueAsDouble()
        + sdsDrivebase.getModule(3).getSteerMotor().getSupplyCurrent().getValueAsDouble();
  }

    public double getDrivebaseCurrent() {
        return sdsDrivebase.getModule(0).getDriveMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(0).getSteerMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(1).getDriveMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(1).getSteerMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(2).getDriveMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(2).getSteerMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(3).getDriveMotor().getSupplyCurrent().getValueAsDouble()
            + sdsDrivebase.getModule(3).getSteerMotor().getSupplyCurrent().getValueAsDouble();
    }

    public void drive(double throttle, double strafe, double rotation) {
        if (this.lastThrottle == throttle && this.lastStrafe == strafe &&
                this.lastRotation == rotation) {
            return;
                }

        if (fieldRelativeDriving) {
            sdsDrivebase.setControl(
                    driveField.withVelocityX(throttle * maxSpeed)
                    .withVelocityY(strafe * maxSpeed)
                    .withRotationalRate(rotation * maxRotRate));
        } else {
            sdsDrivebase.setControl(
                    driveRobot.withVelocityX(throttle * maxSpeed)
                    .withVelocityY(strafe * maxSpeed)
                    .withRotationalRate(rotation * maxRotRate));
        }
        this.lastThrottle = throttle;
        this.lastStrafe = strafe;
        this.lastRotation = rotation;

  }

  public void driveRobotOriented(double throttle, double strafe, double rotation) {
    sdsDrivebase.setControl(
        driveRobot.withVelocityX(throttle * maxSpeed)
            .withVelocityY(strafe * maxSpeed)
            .withRotationalRate(rotation * maxRotRate));

  }

  public void driveToNearestOpening() {
    double targetY = robotPose.getY();
    if (robotPose.getY() >= 1.7 && robotPose.getY() <= 4) {
      targetY = 1.6;
    } else if (robotPose.getY() > 4 && robotPose.getY() <= 6.3) {
      targetY = 6.4;
    }
    // driveToPose(new Pose2d(robotPose.getX(), targetY,
    // Rotation2d.fromDegrees(0.0)), 0.0);
  }

  public void driveMeter() {
    driveToPose(new Pose2d(robotPose.getX() + 0.5, robotPose.getY(), robotPose.getRotation()), 0.0);
  }

  public void driveToPose(Pose2d targetPose, double targetEndVelocity) {
    createPathfindingCommand(targetPose, targetEndVelocity).execute();
  }

  public Command createPathfindingCommand(Pose2d targetPose, double targetEndVelocity) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        Constants.DriveConstants.kMaxSpeed, Constants.DriveConstants.kMaxAcceleration,
        Units.degreesToRadians(Constants.DriveConstants.kMaxAngularRate),
        Units.degreesToRadians(Constants.DriveConstants.kMaxAngularAcceleration));

    // If AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        targetEndVelocity // Goal end velocity in meters/sec
    );
  }

  public void stop() {
    drive(0, 0, 0);
  }

  /**
   * Set drive wheels to X configuration to lock robot from moving.
   */
  public void setX() {
    SwerveRequest xReq = new SwerveRequest.SwerveDriveBrake();
    sdsDrivebase.setControl(xReq);
  }

  public void toggleFieldRelativeDriving() {
    fieldRelativeDriving = !fieldRelativeDriving;

    Util.consoleLog("%b", fieldRelativeDriving);

    updateDS();
  }

  public void toggleSlowMode() {
    slowMode = !slowMode;

    Util.consoleLog("%b", slowMode);

    if (slowMode) {
      // In slow mode, apply slow mode percentages.
      maxSpeed = kMaxSpeed * kSlowModeLinearPct;
      maxRotRate = kMaxAngularRate * kSlowModeRotationPct;
    } else {
      // In normal mode, apply reduction percentages.
      maxSpeed = kMaxSpeed * kDriveReductionPct;
      maxRotRate = kMaxAngularRate * kRotationReductionPct;
    }
  }

  public void toggleNeutralMode() {
    neutralModeBrake = !neutralModeBrake;

    Util.consoleLog("%b", neutralModeBrake);

    if (neutralModeBrake)
      sdsDrivebase.configNeutralMode(NeutralModeValue.Brake);
    else
      sdsDrivebase.configNeutralMode(NeutralModeValue.Coast);

    updateDS();
  }

  public void resetFieldOrientation() {
    Util.consoleLog();

    sdsDrivebase.seedFieldCentric();
  }

  /**
   * Sets the gyroscope yaw angle to zero. This can be used to set the direction
   * the robot is currently facing to the 'forwards' direction.
   */
  public void zeroGyro() {
    Util.consoleLog();

    pigeonWrapper.reset();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    Util.consoleLog(pose.toString());

    sdsDrivebase.resetPose(pose);

    setStartingGyroYaw(-pose.getRotation().getDegrees());
  }

  /**
   * Set a starting yaw for the case where robot is not starting
   * with back bumper parallel to the wall.
   * 
   * @param degrees - is clockwise (cw or right).
   */
  public void setStartingGyroYaw(double degrees) {
    pigeonWrapper.setStartingGyroYaw(degrees);
  }

  /**
   * Returns current sds drivebase odometry pose of the robot.
   * 
   * @return Robot odometry pose.
   */
  public Pose2d getODPose() {
    return sdsDrivebase.getState().Pose;
  }

    /**
     * Returns current pose estimate for the robot.
     * In this function the radians to radians conversion is because the input is
     * incorrect somewhere
     * 
     * @return Robot pose.
     */
    public Pose2d getPose() {
        return getODPose();
    }

  // Get the sds ordometry rotation velocity in radians per second
  public double getRotVelocity() {
    return Math.toDegrees(sdsDrivebase.getStateCopy().Speeds.omegaRadiansPerSecond);
  }

  // Get the sds ordometry x velocity in meters per second
  public double getXVelocity() {
    return sdsDrivebase.getStateCopy().Speeds.vxMetersPerSecond;
  }

  public double getTotalVelocity() {
      return Math.sqrt(Math.pow(getXVelocity(), 2) + Math.pow(getYVelocity(), 2));
  }

  // Get the sds ordometry y velocity in meters per second
  public double getYVelocity() {
    return sdsDrivebase.getStateCopy().Speeds.vyMetersPerSecond;
  }

  @Deprecated
  public Pose3d getPose3d() {
    // I think it should be fine to always assume zero z
    return new Pose3d(getPose());
  }

  public double getYaw() {
    return pigeonWrapper.getYaw();
  }

  public double getYaw180() {
    return pigeonWrapper.getYaw180();
  }

  // This is for setting the Limelight 4s starting position
  public RobotOrientation getRobotOrientation() { // As far as I can tell these are the correct values
    return new RobotOrientation(pigeonWrapper.pigeon.getYaw().getValueAsDouble(),
        pigeonWrapper.pigeon.getAngularVelocityXWorld().getValueAsDouble(),
        pigeonWrapper.pigeon.getPitch().getValueAsDouble(),
        pigeonWrapper.pigeon.getAngularVelocityYDevice().getValueAsDouble(),
        pigeonWrapper.pigeon.getRoll().getValueAsDouble(),
        pigeonWrapper.pigeon.getAngularVelocityZWorld().getValueAsDouble());
  }

  private void updateDS() {
    SmartDashboard.putBoolean(Constants.SmartDashboardKeys.FIELD_ORIENTED, fieldRelativeDriving);
  }

  //
  public void addLimelightMeasurement(Pose2d pose, double timestampSeconds) {
    this.limelightPoseEstimate = pose;
    Pose2d drivebasePose = getPose();

    if (Math.abs(drivebasePose.getX()) > Constants.FIELD_MAX_X || Math.abs(drivebasePose.getY()) > Constants.FIELD_MAX_Y) {
        resetOdometry(pose);
    }

    if (this.bumpHappened) {
        resetOdometry(pose);
        this.bumpHappened = false;
    }

    this.sdsDrivebase.addVisionMeasurement(pose, timestampSeconds);
  }


  public void addQuestPose(Pose2d pose, double timestampSeconds) {

      this.sdsDrivebase.addVisionMeasurement(pose, timestampSeconds);
  }
  /**
   * This function converts an ideal target position into an angle for the robot
   * to face accounting for velocity
   * <p>
   * {@link #getPoseToAim} is secondary to this function, and only returns the
   * position accounted for velocity and not the angle to face
   * 
   * @param targetPose A Pose2d where you want to aim
   * @return An angle (0-360) for the robot to aim, accounting for velocity
   */
  public double getAngleToAim(Pose2d currentPose, Pose2d targetPose) {
    double deltaX = targetPose.getX() - currentPose.getX();
    double deltaY = targetPose.getY() - currentPose.getY();

    double angleToAim;
    angleToAim = Math.toDegrees(Math.atan2(deltaY, deltaX));

    return angleToAim;
  }

  // Get the distance in meters between the current robot position and the target
  // position
  public double getDistFromRobot(Pose2d targetPose) {
    Pose2d currentPose = getPose();

    double deltaX = targetPose.getX() - currentPose.getX();
    double deltaY = targetPose.getY() - currentPose.getY();

    double distance = Math.hypot(deltaX, deltaY);

    return distance;
  }

  /**
   * Update the robot & swerve module displays on the "Field2d" field display in
   * sim.
   * 
   * @param sdsDriveBase Reference to SDS drivebase class under our DriveBase
   *                     class.
   * 
   *                     SDS code has it's own class for logging and doing the
   *                     field display ("Pose") under sim
   *                     and other data logged to the dashboard and to log files
   *                     (Telemetry).
   * 
   *                     We also have our own code for doing the field display and
   *                     it draws the swerve modules
   *                     and thier angles on the simulated robot. This can be
   *                     handy so we are displaying both fields
   *                     for the time being. This function drives that display
   *                     called Field2d under SmartDashboard.
   *                     If this field display is not needed it can be commented
   *                     out and just use the SDS Telemetry.
   */
  @SuppressWarnings("rawtypes")
  private void updateModulePoses(CommandSwerveDrivetrain sdsDriveBase) {
    Pose2d modulePoses[] = new Pose2d[4], robotPose = getPose();

    Translation2d moduleLocations[] = sdsDriveBase.getModuleLocations(), moduleLocation;

    SwerveModule modules[] = sdsDriveBase.getModules();

    for (int i = 0; i < modules.length; i++) {
      moduleLocation = moduleLocations[i]
          .rotateBy(robotPose.getRotation())
          .plus(robotPose.getTranslation());

      modulePoses[i] = new Pose2d(moduleLocation,
          modules[i].getCurrentState().angle.plus(Rotation2d.fromDegrees(-getYaw180())));
    }

    field2d.getObject("Robot").setPose(new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(Math.toRadians(robotPose.getRotation().getRadians()))));
    field2d.getObject("Swerve Modules").setPoses(modulePoses);
  }

  public void toggleHubTracking() {
    Constants.HUB_TRACKING = !Constants.HUB_TRACKING;
  }

  public void enableHubTracking() {
    Constants.HUB_TRACKING = true;
  }

  public void disableHubTracking() {
    Constants.HUB_TRACKING = false;
  }

  public void stopHumanDriving() {
    driverControlled = false;
  }

  public void startHumanDriving() {
    driverControlled = true;
  }

  public boolean getDriverControled() {
    return driverControlled;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return sdsDrivebase.getChassisSpeeds();
  }

  public void enabledSlowMode() {
    this.slowMode = true;
  }

  public void disableSlowMode() {
    this.slowMode = false;
  }

  public ChassisSpeeds getSpeeds() {
    return sdsDrivebase.getSpeeds();
  }

  public void setWallTrackingLeft() {
      this.wallTrackingLeft = true;
  }

  public void setWallTrackingRight() {
      this.wallTrackingRight = true;
  }

  public void clearWallTacking() {
      this.wallTrackingLeft = false;
      this.wallTrackingRight = false;
  }

  public void setBumpHappened() {
      this.bumpHappened = true;
  }
}
