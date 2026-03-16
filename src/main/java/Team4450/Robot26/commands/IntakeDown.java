package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import Team4450.Robot26.Constants;

public class IntakeDown extends Command {

  Intake intake;

  public IntakeDown(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
    SmartDashboard.putNumber(Constants.SmartDashboardKeys.PIVIT_POSiTION, 0.95);
  }

  public void execute() {
  }

  public boolean isFinished() {
    return intake.getPivitPosition() > 0.8;
  }

  @Override
  public void end(boolean inturrupted) {

  }
}
