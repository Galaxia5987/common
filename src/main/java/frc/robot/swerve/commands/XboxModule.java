package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class XboxModule extends CommandBase {

    private final ModuleIO io;
    private final SwerveModule swerveModule;
    private final XboxController xboxController;

    public XboxModule(ModuleIO io, XboxController xboxController){
        this.io = io;
        this.xboxController = xboxController;
        swerveModule = new SwerveModule(io, 1);
        addRequirements(swerveModule);
    }

    @Override
    public void execute() {
        swerveModule.setVelocity(
                MathUtil.applyDeadband(-xboxController.getLeftY(), SwerveConstants.XBOX_DEADBAND)
        );

        swerveModule.setAngleSpeed(
                MathUtil.applyDeadband(-xboxController.getRightX(), SwerveConstants.XBOX_DEADBAND)
        );
    }
}
