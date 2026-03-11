package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import Team4450.Lib.Util;
import Team4450.Robot26.subsystems.Drivebase;
import static Team4450.Robot26.Constants.*;

public class PIDDriveCommand extends Command {
    private final Drivebase drivebase;

    public final PIDController throttlePID;
    public final PIDController strafePID;
    public final PIDController headingPID;

    public double targetX;
    public double targetY;
    public double targetHeading;

    public PIDDriveCommand(Drivebase drivebase, PIDController throttlePID, PIDController strafePID, PIDController headingPID, double targetX, double targetY, double targetHeading) {
        Util.consoleLog();

        this.drivebase = drivebase;
        this.throttlePID = throttlePID;
        this.strafePID = strafePID;
        this.headingPID = headingPID;

        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeading;

        headingPID.setIntegratorRange(-ROBOT_HEADING_KI_MAX, ROBOT_HEADING_KI_MAX);
        headingPID.enableContinuousInput(-180, 180);
        headingPID.setTolerance(ROBOT_HEADING_TOLERANCE_DEG);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        if (robot.isAutonomous()) return;
        
        double throttle = throttlePID.calculate(drivebase.getPose().getX(), targetX);
        double strafe = strafePID.calculate(drivebase.getPose().getY(), targetY);
        double rotation = headingPID.calculate(drivebase.getYaw180(), targetHeading);
        drivebase.drive(throttle, strafe, rotation);
    }

    @Override 
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
