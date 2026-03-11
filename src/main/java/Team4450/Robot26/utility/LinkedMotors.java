package Team4450.Robot26.utility;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.List;

public class LinkedMotors {
    private final TalonFX masterMotor;
    private final List<TalonFX> slaveMotors;
    
    public LinkedMotors(TalonFX master, TalonFX... slaves) {
        this.masterMotor = master;
        this.slaveMotors = List.of(slaves);
    }

    /**
     * Gets the total number of linked motors.
     * @return The total number of motors.
     */
    public int getTotalMotors() {
        return this.slaveMotors.size() + 1;
    }

    /**
     * Gets a motor by its index.
     * @param i The index of the motor.
     * @return The motor at the specified index, or null if out of bounds.
     */
    public TalonFX getMotorByIndex(int i) {
        if (i < 0 || i >= this.slaveMotors.size() + 1) {
            return null;
        }
        return i == 0 ? this.masterMotor : this.slaveMotors.get(i - 1);
    }
}
