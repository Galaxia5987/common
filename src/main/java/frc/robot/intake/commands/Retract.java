package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;

public class Retract extends Command {

    private final Intake intake = Intake.getInstance();

    private final double spinPower;
    private final double anglePower;

    public Retract(double spinPower, double anglePower) {
        this.spinPower = spinPower;
        this.anglePower = anglePower;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setSpinPower(spinPower);
        intake.setAnglePower(anglePower);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAnglePower(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getAngleAppliedCurrent() > 40;
    }
}
