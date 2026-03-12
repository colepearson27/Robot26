package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Drivebase;
import Team4450.Robot26.subsystems.Hopper;
import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: Untested and Unreviewed
public class ShootWithX extends Command {
  private Shooter shooter;
  private Hopper hopper;
  private Drivebase drivebase;

  public ShootWithX(Drivebase drivebase, Shooter shooter, Hopper hopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.drivebase = drivebase;
  }

    @Override
    public void initialize() {
        shooter.enabledHood();
        shooter.startFlywheel();
        drivebase.setX();
    }

    @Override
    public void execute() {
        if (this.shooter.flywheelAtSpeed()) {
            shooter.startInfeed();
            // drivebase.setX();
            hopper.start();
        } else {
            shooter.stopInfeed();
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
        drivebase.drive(0, 0, 0);
        shooter.stopFlywheel();
        shooter.stopInfeed();
        hopper.stop();
    }
}
