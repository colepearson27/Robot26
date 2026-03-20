package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Drivebase;
import Team4450.Robot26.subsystems.Hopper;
import Team4450.Robot26.subsystems.Shooter;
import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
  private Shooter shooter;
  private Hopper hopper;
  private Drivebase drivebase;
  private Intake intake;
  private Timer timer;

  public Shoot(Drivebase drivebase, Shooter shooter, Hopper hopper, Intake intake) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.drivebase = drivebase;
    this.intake = intake;
    this.timer = new Timer();
  }

    @Override
    public void initialize() {
        shooter.enabledHood();
        shooter.startFlywheel();
        drivebase.setX();
        timer.start();
        timer.reset();
        intake.shootingPivitToggle();
    }

    @Override
    public void execute() {
        drivebase.setX();
        if (this.shooter.flywheelAtSpeed()) {
            shooter.startInfeed();
            hopper.start();
        }

        if (!this.shooter.flywheelWithinSpeed()) {
            shooter.stopInfeed();
        }

        if(timer.hasElapsed(0.5)){
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
        shooter.distableHood();
        shooter.stopFlywheel();
        shooter.stopInfeed();
        hopper.stop();
        
    }
}
