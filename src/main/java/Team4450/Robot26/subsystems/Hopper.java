package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.CANBus;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    private final TalonFX hopperMotorRight = new TalonFX(Constants.HOOD_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    private RobotContainer robotContainer;
    public Hopper() {
        // Configure motor neutral mode
        hopperMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration hopperCFG = new TalonFXConfiguration();

        // Neutral + inversion
        hopperCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperCFG.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.LOWER_ROLLERS_CURRENT_LIMIT);
        hopperCFG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        this.hopperMotor.getConfigurator().apply(hopperCFG);
        this.hopperMotorRight.getConfigurator().apply(hopperCFG);

        this.hopperMotor.set(0);
        this.hopperMotorRight.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lower Rollers Current Draw", getLowerRollersCurrent());
    }

    public void start() {
        hopperMotor.set(1);
        hopperMotorRight.setControl(new Follower(this.hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void startWithScaling() {
        hopperMotor.set(1 * robotContainer.getVolatgePercent() * Constants.HOPPER_VOLTAGE_MULTIPLIER);
        hopperMotorRight.setControl(new Follower(this.hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void startSlow() {
        hopperMotor.set(0.2);
        hopperMotorRight.setControl(new Follower(this.hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void stop() {
        hopperMotor.set(0);
        hopperMotorRight.set(0);
    }

    public double getLowerRollersCurrent() { // TODO: FIX
        return hopperMotor.getSupplyCurrent(true).getValueAsDouble();
    }
}
