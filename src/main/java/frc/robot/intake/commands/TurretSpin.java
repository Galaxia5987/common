package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;

public class TurretSpin extends Command {

    private final Intake intake = Intake.getInstance();

    private final double speed;

    public TurretSpin(double spinPower) {
        this.speed = spinPower;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setTurretSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTurretSpeed(0);
    }
}
