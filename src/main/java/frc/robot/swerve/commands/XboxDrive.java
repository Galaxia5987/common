package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveConstantsNeo;
import frc.robot.swerve.SwerveDrive;

public class XboxDrive extends Command {
    private final SwerveDrive swerveDrive;
    private final XboxController xboxController;

    public XboxDrive(SwerveDrive swerveDrive, XboxController xboxController) {
        this.swerveDrive = swerveDrive;
        this.xboxController = xboxController;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                MathUtil.applyDeadband(-xboxController.getLeftY(), SwerveConstantsNeo.XBOX_DEADBAND),
                MathUtil.applyDeadband(-xboxController.getLeftX(), SwerveConstantsNeo.XBOX_DEADBAND),
                MathUtil.applyDeadband(
                        -xboxController.getRightX() * SwerveConstantsNeo.STEERING_MULTIPLIER,
                        SwerveConstantsNeo.XBOX_DEADBAND),
                true);
    }
}
