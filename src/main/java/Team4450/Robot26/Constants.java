package Team4450.Robot26;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Properties;

import Team4450.Robot26.subsystems.SDS.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static String		PROGRAM_NAME = "ORF26-02.12.26";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;

    public static final double  ROBOT_PERIOD_SEC = .02;
    public static final int     ROBOT_PERIOD_MS = 20;

	// Non-drive base motor controller port assignments
    
    // Misc port assignments.
    public static final int     REV_PDB = 20;
    public static final int     CTRE_CANDLE = 21;
    public static final int     PIGEON_ID = 1;

    // Canivore device name or "rio" if no canivore.
    public static String        CANIVORE_NAME = "canivore";
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;

    public static int KRAKEN_X60_MAX_THEORETICAL_RPM = 6000;
    public static int KRAKEN_X44_MAX_THEORETICAL_RPM = 7530;

    // Shooter Constants
    public static int FLYWHEEL_MOTOR_TOP_LEFT_CAN_ID = 13;
    public static int FLYWHEEL_MOTOR_TOP_RIGHT_CAN_ID = 14;
    public static int FLYWHEEL_MOTOR_BOTTOM_RIGHT_CAN_ID = 16;
    public static int FLYWHEEL_MOTOR_BOTTOM_LEFT_CAN_ID = 15;

    public static int HOOD_MOTOR_LEFT_CAN_ID = 17;
    public static int HOOD_MOTOR_RIGHT_CAN_ID = 18;

    public static int INFEED_MOTOR_LEFT_CAN_ID = 19;
    public static int INFEED_MOTOR_RIGHT_CAN_ID = 22;

    public static int SHOOTER_UPPER_BEAM_BREAK_PORT = 3;

    public static int SHOOTER_FLYWHEEL_CURRENT_LIMIT = 35;
    public static int SHOOTER_INFEED_CURRENT_LIMIT = 40;
    public static int SHOOTER_HOOD_CURRENT_LIMIT = 5;
    public static int INTAKE_CURRENT_LIMIT = 20;
    public static int INTAKE_PIVIT_CURRENT_LIMIT = 5;
    public static int LOWER_ROLLERS_CURRENT_LIMIT = 30;

    public static double HOOD_GEAR_RATIO = 3.0/8.0;
    public static double HOOD_DOWN_ANGLE_DEGREES = 15;
    public static double HOOD_MOTOR_POWER = 0.1;


    // Hood PID / Feedforward / MotionMagic
    public static final double HOOD_kP = 15;
    public static final double HOOD_kI = 0;
    public static final double HOOD_kD = 0;
    public static final double HOOD_kS = 0.47;
    public static final double HOOD_kV = 0.1;
    public static final double HOOD_kA = 0.1;
    public static final double HOOD_MOTION_ACCEL = 5.0;
    public static final double HOOD_MOTION_JERK = 0.0;

    // Intake Constants
    public static int INTAKE_MOTOR_PIVIT_CAN_ID = 9;
    public static int INTAKE_MOTOR_LEFT_CAN_ID = 10;
    public static int INTAKE_MOTOR_RIGHT_CAN_ID = 11;

    // The intake gear ratio it 1 to 1
    public static int INTAKE_GEAR_RATIO = 1 / 1;
    // I was told that the gear box on the Kraken is a 25:1
    // public static int INTAKE_PIVIT_GEAR_RATIO = (25 / 1) * (32 / 16);
    public static double INTAKE_PIVIT_GEAR_RATIO = (62.4 / 1);

    public static int INTAKE_MAX_THEORETICAL_RPM = KRAKEN_X44_MAX_THEORETICAL_RPM / INTAKE_GEAR_RATIO;
    public static double INTAKE_MAX_THEORETICAL_PIVIT_RPM = KRAKEN_X60_MAX_THEORETICAL_RPM / INTAKE_PIVIT_GEAR_RATIO;

    public static double INTAKE_PIVIT_MOTOR_POWER = 0.0;
    // The format of this value is in rotations of the pivit motor
    public static int INTAKE_PIVIT_MOTOR_POSITION_UP = 0;
    // This is an assumed value and not exact
    public static double INTAKE_PIVIT_POSITION_DOWN_DEGREES = 105;
    // The format of this value is in rotations of the pivit motor
    public static double INTAKE_PIVIT_MOTOR_POSITION_DOWN = (INTAKE_PIVIT_POSITION_DOWN_DEGREES / 360) * INTAKE_PIVIT_GEAR_RATIO;
    // The format of this value is in rotations of the pivit motor
    public static double INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS = 0.5;
    public static double INTAKE_PIVIL_TOLERENCE_DEGREES = 360 * (INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS / INTAKE_PIVIT_GEAR_RATIO);

    public static String LIMELIGHT_FRONT = "limelight-front";
    public static String LIMELIGHT_RIGHT = "limelight-right";

    // The rotation values are in radians

    public static Pose3d ROBOT_TO_QUEST = new Pose3d(0, 0, 0, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

    public static Pose3d ROBOT_TO_LIMELIGHT_FRONT = new Pose3d(0.305, 0, 0, new Rotation3d(0, -0.3491, 0));
    public static Pose3d ROBOT_TO_LIMELIGHT_RIGHT = new Pose3d(0.0762, 0.318, 0, new Rotation3d(0, -0.3491, -1.5708));
    public static int VISION_BUFFER_SIZE = 1;

    public static double LIMELIGHT_QUEST_ERROR_AMOUNT_METERS = 0.2;

    // Voltage Modifiers
    public static double MAX_BATTERY_VOLTAGE = 13.5;
    public static double INFEED_VOLTAGE_MULTIPLIER = 1.0;
    public static double HOPPER_VOLTAGE_MULTIPLIER = 1.0;
    public static double INTAKE_VOLTAGE_MULTIPLIER = 1.0;
    public static double FLYWHEEL_VOLTAGE_MULTIPLIER = 1.0;

    // Assume all field measurements are in meters
    // Field Limits (The Origin of the field should be the bottom left corner therefore all pose should be in +, +)
    public static double FIELD_MAX_X = 16.54;
    public static double FIELD_MAX_Y = 8.07;

    // HUB Positions (Center of the HUB)
    
    public static double HUB_BLUE_X = 4.625;
    public static double HUB_BLUE_Y = 4.034;
    // Red
    public static double HUB_RED_X = 9.375;
    public static double HUB_RED_Y = 4.034;

    // Blue
    // Comment out the not welded field
    // public static Pose2d HUB_BLUE_ANDYMARK_POSE = new Pose2d(4.611, 4.021, Rotation2d.kZero);
    public static Pose2d HUB_BLUE_WELDED_POSE = new Pose2d(4.625, 4.034, Rotation2d.kZero);
    // Red
    // public static Pose2d HUB_RED_ANDYMARK_POSE = new Pose2d(11.901, 4.021, Rotation2d.kZero);
    public static Pose2d HUB_RED_WELDED_POSE = new Pose2d(11.915, 4.034, Rotation2d.kZero);

    public static double NEUTRAL_BLUE_ZONE_BARRIER_X = 4.572;
    public static double NEUTRAL_RED_ZONE_BARRIER_X = 11.938;

    public static Pose2d FERRY_BLUE_OUTPOST_CORNER = new Pose2d(1.27, 0.635, Rotation2d.kZero);
    public static Pose2d FERRY_BLUE_BLANK_CORNER = new Pose2d(1.27, 7.407, Rotation2d.kZero);
    
    public static Pose2d FERRY_RED_OUTPOST_CORNER = new Pose2d(15.243, 7.407, Rotation2d.kZero);
    public static Pose2d FERRY_RED_BLANK_CORNER = new Pose2d(15.243, 0.635, Rotation2d.kZero);

    public static double FIELD_MIDDLE_Y = 4.021;

    public static double ROBOT_HEADING_KP = 0.02;
    public static double ROBOT_HEADING_KI = 0;
    public static double ROBOT_HEADING_KI_MAX = 0;
    public static double ROBOT_HEADING_KD = 0.001;
    public static double ROBOT_HEADING_TOLERANCE_DEG = 0.0001;
    public static boolean HUB_TRACKING = false;

    // Interpolation table
    public static double[] FLYWHEEL_SPEED_TABLE = {3550, 3650, 3850, 4050, 4300, 4400}; // Converted from percentages to RPM
    public static double[] FLYWHEEL_SPEED_DISTANCE_TABLE = {2, 2.5, 3, 3.5, 4, 4.5};
    public static double[] HOOD_ARC_TABLE = {0.9, 1.65, 1.8, 1.85, 1.85, 2.3};

    public static double[] FUEL_AIR_TIME_TABLE_SEC = {1.1, 1.3, 1.4, 1.5, 1.8, 1.9, 2.1};

    // -------------------------------------------------------------------------------------
    // Flywheel tuning defaults (used as Shuffleboard starting values)
    
    // Default target RPM
    // public static final double FLYWHEEL_TARGET_RPM = 2650.0;
    public static final double FLYWHEEL_TARGET_RPM = 2000;

    // CAN ID for flywheel TalonFX
    public static final int FLYWHEEL_MOTOR_CAN_ID = -1;

    // Closed-loop slot selection
    public static final int FLYWHEEL_PID_SLOT = 0;

    // ---------------- Feedforward (Talon internal) ----------------
    // Units: Volts, Volts/(rps), Volts/(rps/s)
    public static final double FLYWHEEL_kS = 0.1;
    public static final double FLYWHEEL_kV = 0.12;
    public static final double FLYWHEEL_kA = 0.05;
    // ---------------- PID (Velocity) ----------------
    public static final double FLYWHEEL_kP = 0.2;
    public static final double FLYWHEEL_kI = 0;
    public static final double FLYWHEEL_kD = 0;

    // ---------------- Motion Magic Velocity ----------------
    // These only affect ramp rate

    public static final double FLYWHEEL_MOTION_ACCEL_RPMS = 3500.0; // RPM/s
    public static final double FLYWHEEL_MOTION_JERK = 0.0;

    // ---------------- Telemetry / limits ----------------
    public static final double FLYWHEEL_MAX_THEORETICAL_RPM = 6000.0;

    // Flip this to 1 or -1 if direction is wrong
    public static final int FLYWHEEL_DIRECTION = -1;

    // Hopper motor constants
    public static final int HOPPER_MOTOR_CAN_ID = 12; // Example CAN ID for the Kraken X60 motor

    public static final int INTAKE_DEFAULT_TARGET_RPM = 5500;
    // PID constants for Intake
    public static final double INTAKE_kP = 0.8;

    public static final int INFEED_DEFAULT_TARGET_RPM = 5500;
    // PID constants for Shooter Infeed
    public static final double INFEED_kP = 0.8;

    // SmartDashboard key constants for Shooter
    public static final class SmartDashboardKeys {
        // Hood
        public static final String HOOD_POSITION        = "Hood Position";
        public static final String HOOD_ANGLE           = "Hood Angle";
        public static final String HOOD_MOTOR_POSITION  = "Hood Motor Position";
        public static final String HOOD_TARGET_POSITION  = "Hood Target Position";
        public static final String HOOD_POWER           = "Hood Power";

        // Flywheel telemetry
        public static final String FLYWHEEL_TARGET_RPM      = "Flywheel/TargetRPM";
        public static final String FLYWHEEL_MEASURED_RPM    = "Flywheel/MeasuredRPM";
        public static final String FLYWHEEL_MEASURED_RPM_LEGACY = "Flywheel measured RPM";
        public static final String FLYWHEEL_CURRENT_DRAW    = "Flywheel Current Draw";

        // Flywheel PID / FF tuning
        public static final String FLYWHEEL_KP = "Flywheel/kP";
        public static final String FLYWHEEL_KI = "Flywheel/kI";
        public static final String FLYWHEEL_KD = "Flywheel/kD";
        public static final String FLYWHEEL_KS = "Flywheel/kS";
        public static final String FLYWHEEL_KV = "Flywheel/kV";
        public static final String FLYWHEEL_KA = "Flywheel/kA";

        // Infeed
        public static final String INFEED_TARGET_RPM  = "Infeed Target RPM";
        public static final String INFEED_RPM         = "Infeed RPM";
        public static final String INFEED_CURRENT_DRAW = "Infeed Current Draw";

        // Misc
        public static final String BEAM_BREAK                       = "Beam Break";
        public static final String DISABLE_AUTO_FLYWHEEL_UPDATE     = "disableAutomaticFlywheelUpdate";
        public static final String DISABLE_AUTO_DISTANCE_UPDATE     = "disableAutomaticDistanceUpdate";
        public static final String DISABLE_AUTO_DISTANCE_UPDATE_TWO     = "disableAutomaticDistanceUpdateTwo";
        public static final String DISABLE_AUTO_DISTANCE_UPDATE_THREE     = "disableAutomaticDistanceUpdateThree";
        public static final String ROBOT_LAUNCH_X                   = "RobotLaunchX";
        public static final String ROBOT_LAUNCH_Y                   = "RobotLaunchY";
        public static final String GOAL_POSE                        = "Goal Pose";
        public static final String ROBOT_DISTANCE                   = "Robot Distance";

        // Heading
        public static final String HEADING_P = "Heading P";
        public static final String HEADING_I = "Heading I";
        public static final String HEADING_D = "Heading D";
        public static final String HEADING_PID_TOGGLE = "Heading PID Toggle";
        public static final String HEADING_PID_OUTPUT = "Heading PID Output";
        public static final String TARGET_HEADING = "Target Heading";
        public static final String HEADING_ERROR = "Heading Error";

        // Pivit
        public static final String PIVIT_POSiTION = "Pivit Position";

        // Control Flow
        public static final String DISABLED = "Disabled";
        public static final String AUTO_MODE = "Auto Mode";
        public static final String TELEOP_MODE = "Teleop Mode";
        public static final String PROGRAM = "Program";
        public static final String FMS = "FMS";
        public static final String AUTONOMOUS_ACTIVE = "Autonomous Active";

        // Quest
        public static final String QUEST_CONNECTED = "Quest Connected";
        public static final String QUEST_TRACKING = "Quest Tracking";

        // Vision
        public static final String SEND_FRONT_LIMELIGHT_INFO = "Send Front Limelight info";
        public static final String SEND_RIGHT_LIMELIGHT_INFO = "Send Right Limelight info";

        // Drivebase
        public static final String OVERRIDE_QUEST_FOR_ROBOT_POSE = "overrideQuestForRobotPose";
        public static final String PIGEON_GYRO = "Pigeon Gyro";
        public static final String FIELD2D = "Field2d";
        public static final String BATTERY_VOLTAGE = "Battery Voltage";
        public static final String GYRO_ANGLE = "Gyro angle";
        public static final String ROBOT_OD_POSE = "Robot od pose";
        public static final String ROBOT_POSE = "Robot pose";
        public static final String LIMELIGHT_POSE = "Limelight Pose";
        public static final String DRIVEBASE_CURRENT = "Drivebase Current";
    }

    public static final class DriveConstants {
        // Driving Parameters - These are the maximum capable speeds of the robot.

        // Top speed determined by TunerX. Rotation speed reccommended by CTRE.
        // 2026 robot max speed is 5.29 m/s.
        public static double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
        public static double kMaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 1 rotation per second max angular velocity

        public static double kMaxAcceleration = -1;
        public static double kMaxAngularAcceleration = -1;

        // Velocity dead bands applied in SDS code. Times max speed.
        public static final double  DRIVE_DEADBAND = 0.01, ROTATION_DEADBAND = 0.001;

        // Factors used to reduce robot max speed to levels desired for lab/demo operation.
        // The split below matches the rotation speed to drive speed. Needs to be tuned for
        // full weight robot.
        public static final double kDriveReductionPct = .50; // 50% of max linear speed.
        public static final double kRotationReductionPct = .70; // 70% of max rotational speed.

        // Factors used to slow robot speed for fine driving.
        public static final double kSlowModeLinearPct = .15; // 15% of max linear speed.
        public static final double kSlowModeRotationPct = .40; // 40% of max rotational speed.

        // Drive Motor ramp rate. Needs to be tuned for full weight robot.
        public static final double kDriveRampRate = .5; // 0 to 12v in .5 second.

        // Starting pose for sim. Is lower left corner (blue) or where we want sim robot to start.
        //public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(7.473, .559, Rotation2d.kZero);
        public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(0, 0, Rotation2d.kZero);
        public static final Pose3d	DEFAULT_STARTING_POSE_3D = new Pose3d(0, 0, 0, Rotation3d.kZero);
    }
};
