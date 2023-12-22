package frc.robot.swerve.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;
import frc.robot.utils.controllers.DieterController;

public class DriveTillPitch extends CommandBase {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    private final double desiredPitch;
    private final double xVelocity;

    private final DieterController yawController = new DieterController(3, 0, 0, 0);

    public DriveTillPitch(double desiredPitch, double xVelocity) {
        this.desiredPitch = desiredPitch;
        this.xVelocity = xVelocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                new ChassisSpeeds(
                        xVelocity,
                        0,
                        yawController.calculate(swerveDrive.getYaw(), 0)
                ),
                true
        );
    }

    @Override
    public boolean isFinished() {
        if (desiredPitch > 0) {
            return swerveDrive.getPitch() >= desiredPitch;
        } else if (desiredPitch < 0) {
            return swerveDrive.getPitch() <= desiredPitch;
        } else {
            return Utils.epsilonEquals(swerveDrive.getPitch(), 0, 1);
        }
//         return Math.abs(gyroscope.getPitch().getDegrees()) >= Math.abs(desiredPitch);
    }

    @Override
    public void end(boolean interrupted) {

        swerveDrive.stop();
    }
}
