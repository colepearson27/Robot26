package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import Team4450.Robot26.Constants;

public class IntakeUp extends Command {

  Intake intake;

  public IntakeUp(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
  }

  public void execute() {
    SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVOT_POSITION, 0);
  }

  public boolean isFinished() {
    return intake.getPivitPosition() < 0.2;
  }

  @Override
  public void end(boolean inturrupted) {

  }
}
