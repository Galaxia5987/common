package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;

public class KeyboardDriveSim extends Command {

    private final SwerveDrive drive = SwerveDrive.getInstance();

    private final GenericHID controller = new GenericHID(0);

    public KeyboardDriveSim() {
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xOutput = -controller.getRawAxis(1);
        double yOutput = -controller.getRawAxis(0);
        double omegaOutput = controller.getRawAxis(2);

        drive.drive(
                xOutput,
                yOutput,
                omegaOutput,
                true
        );
    }
}