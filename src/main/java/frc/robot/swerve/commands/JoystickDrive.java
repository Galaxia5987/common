package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;

public class JoystickDrive extends Command {
    private final SwerveDrive swerveDrive;
    private final Joystick joystick1;
    private final Joystick joystick2;

    public JoystickDrive(SwerveDrive swerveDrive, Joystick joystick1, Joystick joystick2) {
        this.swerveDrive = swerveDrive;
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                MathUtil.applyDeadband(-joystick1.getY(), 0.1),
                MathUtil.applyDeadband(-joystick1.getX(), 0.1),
                MathUtil.applyDeadband(-joystick2.getX(), 0.1),
                true
        );
    }
}
