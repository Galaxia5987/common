package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.leds.Leds;

public class Lock extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Leds leds = Leds.getInstance();

    public Lock() {
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.lock();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
