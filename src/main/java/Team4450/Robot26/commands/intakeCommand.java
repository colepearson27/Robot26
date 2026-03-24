package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import Team4450.Robot26.subsystems.Hopper;
import edu.wpi.first.wpilibj.Timer;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class intakeCommand extends Command {
    private Intake intake;
    private Hopper hopper;
    private Timer timer;

    public intakeCommand(Intake intake, Hopper hopper) {
        this.intake = intake;
        this.hopper = hopper;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        intake.setIntakeRPM(Constants.INTAKE_DEFAULT_TARGET_RPM);
        timer.start();
        timer.reset();
    }

    private boolean hopperRunning = false;

    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)){
            if(hopperRunning){
                hopper.stop();
            }
            else{
                hopper.start();
            }
            timer.reset();
            hopperRunning = !hopperRunning;

        }
    }

    @Override
    public boolean isFinished() {
        // we will always manually stop this command
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        hopper.stop();
    }
}
