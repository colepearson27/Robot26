package Team4450.Robot26;

import static Team4450.Robot26.Constants.*;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import Team4450.Robot26.commands.DisableHubTracking;
import Team4450.Robot26.commands.DriveCommand;
import Team4450.Robot26.commands.EnableHubTracking;
import Team4450.Robot26.commands.Shoot;
import Team4450.Robot26.commands.ShootWithX;
import Team4450.Robot26.commands.intakeCommand;
import Team4450.Robot26.commands.StopShoot;
import Team4450.Robot26.commands.StopAuto;
import Team4450.Robot26.commands.IntakeUp;
import Team4450.Robot26.commands.IntakeDown;
import Team4450.Robot26.commands.spinShooter;
import Team4450.Robot26.subsystems.Candle;
import Team4450.Robot26.subsystems.Intake;
import Team4450.Robot26.subsystems.Drivebase;
import Team4450.Robot26.subsystems.Shooter;
import Team4450.Robot26.subsystems.ShuffleBoard;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.MonitorPower;
import Team4450.Lib.Util;
import Team4450.Lib.XboxController;
import Team4450.Robot26.commands.DriveCommand;
import Team4450.Robot26.subsystems.Drivebase;
import Team4450.Robot26.subsystems.QuestNavSubsystem;
import Team4450.Robot26.subsystems.ShuffleBoard;
import Team4450.Robot26.subsystems.VisionSubsystem;
import Team4450.Robot26.subsystems.Hopper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
    // Subsystems.
    public static Drivebase drivebase;
    public static ShuffleBoard shuffleBoard;

    // Vision based subsystems all send data to the drivebase for use
    public static VisionSubsystem visionSubsystem;
    public static QuestNavSubsystem questNavSubsystem;

    public final DriveCommand driveCommand;

    public static Intake intake;
    public static Shooter shooter;
    // public TestSubsystem testSubsystem;

    public static Hopper hopper = new Hopper();

    public static boolean inTestMode = false;

    private static SendableChooser<String> stringAutoChooser;
    // private static SendableChooser<Command> autoChooser;

    // Subsystem Default Commands.

    // Persistent Commands.

    // Some notes about Commands.
    // When a Command is created with the New operator, its constructor is called.
    // When the
    // command is added to the Scheduler to be run, its initialize method is called.
    // Then on
    // each scheduler run, as long as the command is still scheduled, its execute
    // method is
    // called followed by isFinished. If isFinished it false, the command remains in
    // the
    // scheduler list and on next run, execute is called followed by isFinished. If
    // isFinished
    // returns true, the end method is called and the command is removed from the
    // scheduler list.
    // Now if you create another instance with new, you get the constructor again.
    // But if you
    // are re-scheduling an existing command instance (like the ones above), you do
    // not get the
    // constructor called, but you do get initialize called again and then on to
    // execute & etc.
    // So this means you have to be careful about command initialization activities
    // as a persistent
    // command in effect has two lifetimes (or scopes): Class global and each new
    // time the command
    // is scheduled. Note the FIRST doc on the scheduler process is not accurate as
    // of 2020.

    // GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some
    // extra features.
    // Note that button responsiveness may be slowed as the schedulers command list
    // gets longer
    // or commands get longer as buttons are processed once per scheduler run.

    public static XboxController driverController = new XboxController(DRIVER_PAD);
    public static XboxController utilityController = new XboxController(UTILITY_PAD);

    private MonitorPower monitorPowerThread;

    private static PIDController headingPID;

    /**
     * The container for the robot. Contains subsystems, Opertor Interface devices,
     * and commands.
     */
    public RobotContainer() throws Exception {
        // this.testSubsystem = new TestSubsystem();

        // Get information about the match environment from the Field Control System.
        getMatchInformation();

        // Read properties file from RoboRio "disk". If we fail to open the file,
        // log the exception but continue and default to competition robot.

        try {
            robotProperties = Util.readProperties();
        } catch (Exception e) {
            Util.logException(e);
        }

        // Is this the competition or clone robot?
        if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
            isComp = true;
        else
            isClone = true;

        // Invert driving joy sticks Y axis so + values mean forward.
        // Invert driving joy sticks X axis so + values mean right.
        driverController.invertY(true);
        driverController.invertX(true);

        // Create subsystems prior to button mapping.
        shuffleBoard = new ShuffleBoard();

        // The pigeon is setup somewhere in the drivebase function.
        // It is important to note that the pigeon documentation says that the device
        // does not need to be still on boot,
        // however the documentation also says that the drift is worse when started
        // while moving.
        drivebase = new Drivebase();
        visionSubsystem = new VisionSubsystem(drivebase);
        questNavSubsystem = new QuestNavSubsystem(drivebase);

        intake = new Intake();
        shooter = new Shooter(drivebase);

        headingPID = new PIDController(Constants.ROBOT_HEADING_KP, Constants.ROBOT_HEADING_KI, Constants.ROBOT_HEADING_KD);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.HEADING_P, Constants.ROBOT_HEADING_KP);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.HEADING_I, Constants.ROBOT_HEADING_KI);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.HEADING_D, Constants.ROBOT_HEADING_KD);
        SmartDashboard.putBoolean(Constants.SmartDashboardKeys.HEADING_PID_TOGGLE, Constants.HUB_TRACKING);

        // Create any persistent commands.

        // Set any subsystem Default commands.

        // Pathplanner NamedCommands

    NamedCommands.registerCommand("intakeDown", new IntakeDown(intake));
    NamedCommands.registerCommand("intakeUp", new IntakeUp(intake));
    NamedCommands.registerCommand("enableHubTracking", new EnableHubTracking(drivebase, headingPID));
    NamedCommands.registerCommand("disableHubTracking", new DisableHubTracking(drivebase));
    NamedCommands.registerCommand("intake", new intakeCommand(intake, hopper));
    NamedCommands.registerCommand("shoot", new Shoot(drivebase, shooter, hopper, intake));
    NamedCommands.registerCommand("stopShooter", new StopShoot(shooter, hopper));
    NamedCommands.registerCommand("end", new StopAuto(drivebase));
    NamedCommands.registerCommand("spinShooter", new spinShooter(shooter));

        // Set the default drive command. This command will be scheduled automatically
        // to run
        // every teleop period and so use the gamepad joy sticks to drive the robot.

        // We pass the GetY() functions on the Joysticks as a DoubleSuppier. The point
        // of this
        // is removing the direct connection between the Drive and XboxController
        // classes. We
        // are in effect passing functions into the Drive command so it can read the
        // values
        // later when the Drive command is executing under the Scheduler. Drive command
        // code does
        // not have to know anything about the JoySticks (or any other source) but can
        // still read
        // them. We can pass the DoubleSupplier two ways. First is with () -> lambda
        // expression
        // which wraps the getLeftY() function in a DoubleSupplier instance. Second is
        // using the
        // controller class convenience method getRightYDS() which returns getRightY()
        // as a
        // DoubleSupplier. We show both ways here as an example.

        // The joystick controls for driving:
        // Left stick Y axis -> forward and backwards movement (throttle)
        // Left stick X axis -> left and right movement (strafe)
        // Right stick X axis -> rotation
        // Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds
        // object
        // and the odometry pose2d classes.
        // Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is
        // left
        // when standing at alliance wall looking down the field.
        // This is handled here by swapping the inputs. Note that first axis parameter
        // below
        // is the X wheelspeeds input and the second is Y wheelspeeds input.

        // Note that field oriented driving does the movements in relation to the field.
        // So
        // throttle is always down the field and back and strafe is always left right
        // from
        // the down the field axis, no matter which way the robot is pointing. Robot
        // oriented
        // driving movemments are in relation to the direction the robot is currently
        // pointing.

        // Note that the controller instance is passed to the drive command for use in
        // displaying
        // debugging information on Shuffleboard. It is not required for the driving
        // function.
        driveCommand = new DriveCommand(drivebase,
                () -> driverController.getLeftY(),
                driverController.getLeftXDS(),
                driverController.getRightXDS(),
                driverController.getRightYDS(), headingPID);

        drivebase.setDefaultCommand(driveCommand);

        monitorPowerThread = MonitorPower.getInstance();
        monitorPowerThread.start();

        // Start a thread that will wait 30 seconds then disable the missing
        // joystick warning. This is long enough for when the warning is valid
        // but will stop flooding the console log when we are legitimately
        // running without both joysticks plugged in.
        new Thread(() -> {
            try {
                Timer.delay(30);
                DriverStation.silenceJoystickConnectionWarning(true);
            } catch (Exception e) {
            }
        }).start();

        // Configure autonomous routines and send to dashboard.
        // autoChooser = AutoBuilder.buildAutoChooser();
        stringAutoChooser = new SendableChooser<String>();

        for (int i = 0; i < AutoBuilder.getAllAutoNames().size(); i++) {
            stringAutoChooser.addOption(AutoBuilder.getAllAutoNames().get(i), AutoBuilder.getAllAutoNames().get(i));
        }

        SmartDashboard.putData("String Auto", stringAutoChooser);

        // Configure the button bindings.
        configureButtonBindings();

        // Warmup PathPlanner to avoid Java pauses.
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    /**
     * Use this method to define your button->command mappings.
     * 
     * These buttons are for robot driver station with 2 Xbox or F310 controllers.
     */
    private void configureButtonBindings() {
        // ------- Driver controller buttons -------------

        // For simple functions, instead of creating commands, we can call convenience
        // functions on
        // the target subsystem from an InstantCommand. It can be tricky deciding what
        // functions
        // should be an aspect of the subsystem and what functions should be in
        // Commands...

        // POV buttons do same as alternate driving mode but without any lateral
        // Movement and increments of 45deg.
        // new Trigger(()-> driverController.getPOV() != -1)
        // .onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()),
        // driveBase, false))

        // Vibrate between 30 and 25 sec left in match.
        new Trigger(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25).whileTrue(new StartEndCommand(
                () -> {
                    driverController.setRumble(RumbleType.kBothRumble, 0.5);
                    utilityController.setRumble(RumbleType.kBothRumble, 0.5);
                },
                () -> {
                    driverController.setRumble(RumbleType.kBothRumble, 0);
                    utilityController.setRumble(RumbleType.kBothRumble, 0);
                }));

        // Reset field orientation (direction).
        // new Trigger(() -> driverController.getPOV() == 180) // D-pad down Cole
        // .onTrue(new InstantCommand(drivebase::resetFieldOrientation));

        // Toggle field-oriented driving mode.
        // new Trigger(() -> driverController.getAButton()) // Rich
        // .onTrue(new InstantCommand(driveBase::toggleFieldRelativeDriving));

        // new Trigger(() -> driverController.getAButton())
        // .onTrue(new InstantCommand(questNavSubsystem::resetTestPose));

        // new Trigger(() -> driverController.getBButton())
        // .onTrue(new InstantCommand(questNavSubsystem::resetToZeroPose));

        // new Trigger(() -> driverController.getBButton())
        // .onTrue(new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(0, 0,
        // Rotation2d.kZero))));

        // // Toggle motor brake mode.
        // new Trigger(() -> driverController.getBButton()) // Rich
        // .onTrue(new InstantCommand(driveBase::toggleNeutralMode));

        // Toggle slow-mode
        // Right D-Pad button sets X pattern to stop movement.

        new Trigger(() -> driverController.getLeftBumperButton()) // Rich
                .onChange(new InstantCommand(drivebase::toggleSlowMode));

    new Trigger(() -> driverController.getPOV() == 0) // Up D-pad
        .onTrue(new InstantCommand(shooter::toggleManualDistanceOne))
        .onTrue(new InstantCommand(shooter::disableManualDistanceTwo))
        .onTrue(new InstantCommand(shooter::disableManualDistanceThree));

    new Trigger(() -> driverController.getPOV() == 90) // Rich // Right D-pad
        .onTrue(new InstantCommand(drivebase::setX));

    new Trigger(() -> driverController.getPOV() == 180) // Down D-pad
        .onTrue(new InstantCommand(shooter::toggleManaualDistanceThree))
        .onTrue(new InstantCommand(shooter::disableManualDistanceOne))
        .onTrue(new InstantCommand(shooter::disableManualDistanceTwo));

    new Trigger(() -> driverController.getPOV() == 270) // Left D-pad
        .onTrue(new InstantCommand(shooter::toggleManualDistanceTwo))
        .onTrue(new InstantCommand(shooter::disableManualDistanceOne))
        .onTrue(new InstantCommand(shooter::disableManualDistanceThree));
    
    new Trigger(() -> driverController.getRightBumperButton())
        .onTrue(new InstantCommand(intake::togglePivit));

    new Trigger(() -> driverController.getLeftTrigger())
        .whileTrue(new Shoot(drivebase, shooter, hopper, intake));

    new Trigger(() -> driverController.getRightTrigger())
        .onTrue(new InstantCommand(shooter::startInfeed))
        .onFalse(new InstantCommand(shooter::stopInfeed));

    new Trigger(() -> driverController.getAButton())
        .onTrue(new InstantCommand(intake::startIntake))
        .onFalse(new InstantCommand(intake::stopIntake));

    new Trigger(() -> driverController.getBButton())
        .onTrue(new InstantCommand(visionSubsystem::resetYaw))
        .onTrue(new InstantCommand(drivebase::resetFieldOrientation));

    new Trigger(() -> driverController.getYButton())
        .onTrue(new InstantCommand(shooter::reverseInfeed))
        .onTrue(new InstantCommand(intake::reverseIntake))
        .onFalse(new InstantCommand(shooter::stopInfeed))
        .onFalse(new InstantCommand(intake::stopIntake));

    new Trigger(() -> driverController.getXButton())
        .onTrue(new InstantCommand(drivebase::toggleHubTracking));

    new Trigger(() -> driverController.getLeftStickButton())
        .onTrue(new InstantCommand(drivebase::setWallTrackingLeft))
        .onTrue(new InstantCommand(drivebase::enableHubTracking))
        .onFalse(new InstantCommand(drivebase::clearWallTacking))
        .onFalse(new InstantCommand(drivebase::disableHubTracking));

    new Trigger(() -> driverController.getRightStickButton())
        .onTrue(new InstantCommand(drivebase::setWallTrackingRight))
        .onTrue(new InstantCommand(drivebase::enableHubTracking))
        .onFalse(new InstantCommand(drivebase::clearWallTacking))
        .onFalse(new InstantCommand(drivebase::disableHubTracking));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * Determines which auto command from the selection made by the operator on the
     * DS drop down list of commands.
     * 
     * @return The Command to run in autonomous.
     */
    // public Command getAutonomousCommand() {
    // }

    // public static String getAutonomousCommandName() {
    // return autonomousCommandName;
    // }

    // Configure SendableChooser (drop down list on dashboard) with auto program
    // choices and
    // send them to SmartDashboard/ShuffleBoard.

    private void setAutoChoices() {
        // autoChooser = AutoBuilder.buildAutoChooser();

        // SmartDashboard.putData("Auto Program", autoChooser);
    }

    public static String getAutonomousCommand() {
        // return autoChooser.getSelected();
        return stringAutoChooser.getSelected();
    }

    /**
     * Get and log information about the current match from the FMS or DS.
     */
    public void getMatchInformation() {
        alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        location = DriverStation.getLocation().orElse(0);
        eventName = DriverStation.getEventName();
        matchNumber = DriverStation.getMatchNumber();
        gameMessage = DriverStation.getGameSpecificMessage();

        Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s",
                alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber,
                gameMessage);
    }

    public double getVolatgePercent() {
        return RobotController.getBatteryVoltage() / Constants.MAX_BATTERY_VOLTAGE;
    }

    public double getVolatgeMultiplier() {
        return Constants.MAX_BATTERY_VOLTAGE / RobotController.getBatteryVoltage();
    }

    // public void fixPathPlannerGyro() { rich
    // driveBase.fixPathPlannerGyro();
    // }
}
