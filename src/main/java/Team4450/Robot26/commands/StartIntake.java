package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class StartIntake extends Command {
  private Intake intake;

  private State state;

  private static enum State {
    STOPPING, STOPPED, END
  };

  public StartIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    state = State.STOPPING;

  }

  @Override
  public void execute() {
    state = State.STOPPED;
    intake.setIntakeRPM(4500);
    end(false);
  }

  @Override
  public boolean isFinished() {
    return state == State.END;

  }

  @Override
  public void end(boolean interrupted) {

  }
}
