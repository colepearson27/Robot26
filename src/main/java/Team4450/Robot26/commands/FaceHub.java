package Team4450.Robot26.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import java.util.function.DoubleSupplier;
import Team4450.Lib.Util;
import Team4450.Robot26.Constants;
import Team4450.Robot26.subsystems.Drivebase;
import static Team4450.Robot26.Constants.*;

/**
 * While held rotates the robot to face the Hub with odometry heading
 * We can test moving while staying locked on the hub but for now just rotate in place
 * TODO: Test and Tune
 */
public class FaceHub extends Command {

    private final Drivebase drivebase;
    private final PIDController headingPID;
    // private final DoubleSupplier throttleSupplier;
    // private final DoubleSupplier strafeSupplier;

    /**
     * @param drivebase        
     * @param headingPID       
     * @param throttleSupplier 
     * @param strafeSupplier   
     */
    public FaceHub(Drivebase drivebase, PIDController headingPID) { //DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier) {
        Util.consoleLog();

        this.drivebase = drivebase;
        this.headingPID = headingPID;
        // this.throttleSupplier = throttleSupplier;
        // this.strafeSupplier = strafeSupplier;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
        headingPID.enableContinuousInput(-180, 180);
        headingPID.reset();
    }

    @Override
    public void execute() {

        Pose2d hubPosition;
        if (Constants.alliance == DriverStation.Alliance.Blue) {
            hubPosition = new Pose2d(HUB_BLUE_WELDED_POSE.getX(), HUB_BLUE_WELDED_POSE.getY(), Rotation2d.kZero);
        } else {
            hubPosition = new Pose2d(HUB_RED_WELDED_POSE.getX(),  HUB_RED_WELDED_POSE.getY(),  Rotation2d.kZero);
        }

        // gets the target heading with odometry (I think)
        // Pose2d currentPose = drivebase.getODPose(); This uses sds odometry i think thats from the pigeon if we want to use that 
        Pose2d currentPose = drivebase.getPose();   // this uses quest nav
        double deltaX = hubPosition.getX() - currentPose.getX();
        double deltaY = hubPosition.getY() - currentPose.getY();
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double currentHeading = -drivebase.getYaw();
        double rotation = -headingPID.calculate(currentHeading, -targetHeading);

        SmartDashboard.putNumber("FaceHub/TargetHeading", targetHeading);
        SmartDashboard.putNumber("FaceHub/Error", headingPID.getError());

        // lets driver still move while facing the hub but idk if we want that right now tho we probably do
        //double throttle = Util.squareInput(throttleSupplier.getAsDouble());
        //double strafe = Util.squareInput(strafeSupplier.getAsDouble());

       //drivebase.drive(throttle, strafe, rotation);
        drivebase.drive(0, 0, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}