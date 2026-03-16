package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StartIntake extends Command {
  private Intake intake;

  public StartIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.setIntakeRPM(5500);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    // we will always manually stop this command
    return false;

  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }
}
