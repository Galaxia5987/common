package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class JoystickDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Joystick joystick1;
    private final Joystick joystick2;

    private boolean shouldHoldAngle = false;
    private final PIDController omegaController = new PIDController(
            SwerveConstants.OMEGA_kP, SwerveConstants.OMEGA_kI, SwerveConstants.OMEGA_kD);

    public JoystickDrive(SwerveDrive swerveDrive, Joystick joystick1, Joystick joystick2) {
        this.swerveDrive = swerveDrive;
        this.joystick1 = joystick1;
        this.joystick2 = joystick2;
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        shouldHoldAngle = joystick2.getRawButton(Ports.UI.JOYSTICK_RIGHT_BIG_BUTTON);
        if (!shouldHoldAngle) {
            swerveDrive.drive(
                    MathUtil.applyDeadband(-joystick1.getY(), 0.1),
                    MathUtil.applyDeadband(-joystick1.getX(), 0.1),
                    MathUtil.applyDeadband(-joystick2.getX(), 0.1),
                    true
            );
        }
        else {
            swerveDrive.drive(
                    MathUtil.applyDeadband(-joystick1.getY(), 0.1),
                    MathUtil.applyDeadband(-joystick1.getX(), 0.1),
                    omegaController.calculate(swerveDrive.getYaw(), Math.PI),
                    true
            );
        }
    }
}
