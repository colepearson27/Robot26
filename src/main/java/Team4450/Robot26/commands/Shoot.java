package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Drivebase;
import Team4450.Robot26.subsystems.Hopper;
import Team4450.Robot26.subsystems.Shooter;
import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import Team4450.Robot26.Constants;

public class Shoot extends Command {
    private Shooter shooter;
    private Hopper hopper;
    private Drivebase drivebase;
    private Intake intake;
    private Timer timer;
    private Timer pivitDelay;
    private Timer xTimer;
    private Timer infeedDelay;
    private boolean temp;

    public Shoot(Drivebase drivebase, Shooter shooter, Hopper hopper, Intake intake) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.drivebase = drivebase;
        this.intake = intake;
        this.timer = new Timer();
        this.pivitDelay = new Timer();
        this.xTimer = new Timer();
        this.infeedDelay = new Timer();
        this.temp = false;
    }

    @Override
    public void initialize() {
        shooter.enabledHood();
        shooter.startFlywheel();
        drivebase.setX();
        timer.start();
        timer.reset();
        pivitDelay.start();
        pivitDelay.reset();
        xTimer.start();
        xTimer.reset();
        intake.slowIntake();
        infeedDelay.start();
        infeedDelay.reset();
        shooter.enableSlowAcceleration();
    }

    @Override
    public void execute() {
        if (this.xTimer.hasElapsed(0.5)) {
            drivebase.setX();
            this.xTimer.reset();
        }

        if (this.shooter.flywheelAtSpeed()) {
            if (!temp) {
                infeedDelay.reset();
                temp = true;
            }
            if (infeedDelay.hasElapsed(0.2)) {
                hopper.start();
            }
            shooter.startInfeed();
        }

        if (!this.shooter.flywheelWithinSpeed()) {
            SmartDashboard.putNumber(Constants.SmartDashboardKeys.INFEED_TARGET_RPM, (Constants.INFEED_DEFAULT_TARGET_RPM - shooter.flywheelRPMError));
        }

        if (timer.hasElapsed(0.5) && pivitDelay.hasElapsed(2)) {
            intake.shootingPivitToggle();
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        // We will always force stop the command
        return false;
    }

    @Override
    public void end(boolean interuppted) {
        this.temp = false;
        shooter.distableHood();
        shooter.stopFlywheel();
        shooter.stopInfeed();
        shooter.disableSlowAcceleration();
        hopper.stop();
        intake.stopIntake();
    }
}
