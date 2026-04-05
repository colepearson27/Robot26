package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.Constants.SmartDashboardKeys;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.CANBus;

public class Intake extends SubsystemBase {

    // This motor is a Kraken x60
    private final TalonFX pivotMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVOT_CAN_ID,
            new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX intakeMotorLeft = new TalonFX(Constants.INTAKE_MOTOR_LEFT_CAN_ID);
    // This motor is a Kraken x44
    private final TalonFX intakeMotorRight = new TalonFX(Constants.INTAKE_MOTOR_RIGHT_CAN_ID);

    private boolean canPivot;
    private boolean canSpin;

    // This value is expected to be between 0 and 1
    private double pivitTargetPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitTargetPositionMotorPosition;

    // This value is expected to be between 0 and 1
    private double pivitCurrentPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitCurrentPositionMotorPosition;

    private boolean runIntake = false;
    private boolean reverseIntake = false;

    public Intake() {
        this.canPivot = pivotMotor.isConnected();
        this.canSpin = intakeMotorLeft.isConnected() && intakeMotorRight.isConnected();

        // Assume the pivit starting position is 0
        this.pivitCurrentPosition = 0;
        this.pivitTargetPosition = 0;

        if (this.canPivot) {
            this.pivotMotor.setPosition(0);
        }

        TalonFXConfiguration intakeCFG = new TalonFXConfiguration();

        // Neutral + inversion
        intakeCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeCFG.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.SHOOTER_INFEED_CURRENT_LIMIT);
        intakeCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeCFG.Slot0.kP = 0.1;
        intakeCFG.Slot0.kI = 0;
        intakeCFG.Slot0.kD = 0;
        intakeCFG.Slot0.kS = 0.48;
        intakeCFG.Slot0.kV = 0.1;
        intakeCFG.Slot0.kA = 0.1;

        this.intakeMotorLeft.getConfigurator().apply(intakeCFG);
        this.intakeMotorRight.getConfigurator().apply(intakeCFG);

        TalonFXConfiguration pivotCFG = new TalonFXConfiguration();

        // Neutral + inversion
        pivotCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotCFG.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.INTAKE_PIVOT_CURRENT_LIMIT);
        pivotCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Slot 0 PID
        pivotCFG.Slot0.kP = 1;
        pivotCFG.Slot0.kI = 0;
        pivotCFG.Slot0.kD = 0.1;

        // Slot 0 Feedforward (Talon internal)
        pivotCFG.Slot0.kS = 0.5;
        pivotCFG.Slot0.kV = 0;
        pivotCFG.Slot0.kA = 0;

        this.pivotMotor.getConfigurator().apply(pivotCFG);

        SmartDashboard.putBoolean("Intake can Pivot", canPivot);
        SmartDashboard.putBoolean("Intake can Spin", canSpin);
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0);
    }

    @Override
    public void periodic() {
        this.pivitTargetPosition = SmartDashboard.getNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0);
        if (this.canPivot) {
            this.pivitTargetPositionMotorPosition = this.pivitPositionToMotorPosition(this.pivitTargetPosition);
            // Convert position input to rotations for the motor
            // double power = Constants.INTAKE_PIVOT_MOTOR_POWER;

            PositionVoltage req = new PositionVoltage(this.pivitTargetPositionMotorPosition);

            this.pivotMotor.setControl(req);

            this.pivitCurrentPositionMotorPosition = this.getPivitPosition();
            this.pivitCurrentPosition = this.motorPositionToPivitPosition(this.pivitCurrentPositionMotorPosition);

            if (RobotContainer.inTestMode) {
                SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
            }
            SmartDashboard.putNumber("Intake Current Draw", getIntakeCurrent());
            SmartDashboard.putNumber("Intake Pivot Current Draw", getPivotMotorCurrent());
        }

        if (this.runIntake) {
            if (this.reverseIntake) {
                setIntakeRPM(-Constants.INTAKE_DEFAULT_TARGET_RPM);
            } else {
                setIntakeRPM(Constants.INTAKE_DEFAULT_TARGET_RPM);
            }
        }
    }

    public void togglePivit() {
        if (this.pivitCurrentPosition >= 0.8) {
            pivitUp();
        } else {
            pivitDown();
        }
    }

    public void pivitDown() {
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 1.01);
    }

    public void pivitUp() {
        SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0);
    }

    public void shootingPivitToggle() {
        if (this.pivitCurrentPosition >= 0.85) {
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0.40);
        } else {
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0.95);
        }
    }

    public void incrementPivitUp(double incrementAmount) {

        incrementAmount = SmartDashboard.getNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 1) - incrementAmount;

        SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, incrementAmount);
    }

    // Linear interpolate the pivit position between zero and one with the motor
    // rotations of up and down on the pivit
    public double pivitPositionToMotorPosition(double pivitPosition) {
        return Constants.INTAKE_PIVOT_MOTOR_POSITION_UP
                + ((Constants.INTAKE_PIVOT_MOTOR_POSITION_DOWN - Constants.INTAKE_PIVOT_MOTOR_POSITION_UP)
                        * pivitPosition);
    }

    public double motorPositionToPivitPosition(double motorPosition) {
        return (motorPosition / Constants.INTAKE_PIVOT_GEAR_RATIO)
                * (1 / (Constants.INTAKE_PIVOT_POSITION_DOWN_DEGREES / 360));
    }

    public void startIntake() {
        if (canSpin) {
            this.runIntake = true;
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0.95);
        }
    }

    public void reverseIntake() {
        if (canSpin) {
            this.runIntake = true;
            this.reverseIntake = true;
        }
    }

    public void testIntake() {
        this.intakeMotorLeft.set(0.05);
        this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void slowIntake() {
        this.intakeMotorLeft.set(0.30);
        this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void stopIntake() {
        this.runIntake = false;
        this.reverseIntake = false;
        this.intakeMotorLeft.set(0);
        this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public double getIntakeRPM() {
        if (canSpin) {
            return intakeMotorLeft.getRotorVelocity(true).getValueAsDouble() * 60;
        } else {
            return -1;
        }
    }

    public double getIntakeCurrent() {
        return intakeMotorLeft.getSupplyCurrent(true).getValueAsDouble()
                + intakeMotorRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getIntakeLeftMotorCurrent() {
        if (canSpin) {
            return intakeMotorLeft.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorCurrent() {
        if (canSpin) {
            return intakeMotorRight.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public void setPivitMotorSpeed(double speed) {
        if (canPivot) {
            pivotMotor.set(speed);
        }
    }

    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setPivitMotorPosition(double position) {
        pivitTargetPosition = position;
    }

    // TODO:
    public double getPivitPosition() {
        // Need to convert
        if (canPivot) {
            return pivotMotor.getPosition(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getPivotMotorCurrent() {
        if (canPivot) {
            return pivotMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public boolean hasDevices() {
        return pivotMotor.isConnected() && intakeMotorLeft.isConnected() && intakeMotorRight.isConnected();
    }

    /**
     * Sets the power for the intake motors.
     * 
     * @param power The power level to set.
     */
    public void setPower(double power) {
        this.pivotMotor.set(power);
    }

    public void setIntakeRPM(double targetRPM) {
        // double currentRPM = getIntakeRPM();
        // double error = targetRPM - currentRPM;
        // double adjustment = Constants.INTAKE_kP * error; // Adjustment to approach
        // target
        // double newRPM = targetRPM + adjustment; // Adjust current RPM towards target

        VelocityVoltage velReq = new VelocityVoltage(Constants.INTAKE_DEFAULT_TARGET_RPM / 60).withEnableFOC(true);
        this.intakeMotorLeft.setControl(velReq);
        this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }
}
