package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.controllers.DieterController;

public class DriveTillDeltaPitch extends CommandBase {
    private final DieterController yawController = new DieterController(3, 0, 0, 0);
    SwerveDrive swerveDrive = SwerveDrive.getInstance();
    double xVelocity;
    double deadband;
    double pitch;
    double lastPitch = pitch;


    public DriveTillDeltaPitch(double deadband, double xVelocity) {
        this.xVelocity = xVelocity;
        this.deadband = deadband;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        pitch = swerveDrive.getPitch();
        swerveDrive.drive(
                xVelocity,
                0,
                yawController.calculate(swerveDrive.getYaw(), 0),
                false
        );
        lastPitch = pitch;

    }

    @Override
    public boolean isFinished() {
        return MathUtil.applyDeadband(pitch - lastPitch, deadband) == 0;
    }
}
