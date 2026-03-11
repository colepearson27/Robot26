package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Drivebase;

public class DisableHubTracking extends Command {

  Drivebase drivebase;

  public DisableHubTracking(Drivebase drivebase) {

    this.drivebase = drivebase;

  }

  @Override
  public void initialize() {
    drivebase.disableHubTracking();
  }

  @Override
  public void execute() {
    end(false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean inturrupted) {

  }
}
