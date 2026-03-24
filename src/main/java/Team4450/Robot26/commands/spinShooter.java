package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class spinShooter extends Command {
    private Shooter shooter;

    public spinShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.enabledHood();
        shooter.startFlywheel();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // We will always force stop the command
        return false;
    }

    @Override
    public void end(boolean interuppted) {
        shooter.distableHood();
    }
}
