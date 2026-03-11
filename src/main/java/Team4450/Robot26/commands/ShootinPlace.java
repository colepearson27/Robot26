package Team4450.Robot26.commands;

import Team4450.Lib.Util; 
import Team4450.Robot26.subsystems.shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.subsystems.Drivebase;
import edu.wpi.first.math.controller.PIDController;

public class ShootinPlace {
    private Drivebase drivebase;
    private Turret turret;
    double targetAngle;
    private Pose2d currentPose2d;
    PIDController rotationController = new PIDController(0.002, 0, 0);
    private boolean finished = false;
    
    public ShootinPlace(Drivebase drivebase, Turret turret) {
        this.drivebase = drivebase;
        this.turret = turret;
    }

    public void initialize() {
        currentPose2d = drivebase.getPose();
        targetAngle = turret.getAngleToFaceGoalDegrees(currentPose2d);
        Util.consoleLog("Target Angle: %.2f", targetAngle);
        finished = false;
        rotationController.setTolerance(0.5);
        rotationController.setSetpoint(0);

    }
    
    public void execute() {
        double rotation = rotationController.calculate(targetAngle);
        drivebase.drive(0, 0, rotation);
         if (rotation < 0.07) {
            finished = true;
        }
    }

    public boolean isFinished(){
        if(finished){
            return true;
        }
        
        else{
            return false;
        }
    }

     public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        
        drivebase.drive(0, 0, 0);
        
    }
}
