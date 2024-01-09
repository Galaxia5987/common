package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveDriveInputs {
    public double supplyCurrent;
    public double statorCurrent;

    public SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];
    public SwerveModuleState[] desiredModuleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    // x, y, omega
    public ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    public ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    public double linearVelocity = 0;
    public double acceleration = 0;

    public double[] absolutePositions = new double[4];

    public Rotation2d pitch;
    public Rotation2d rawYaw;
    public Rotation2d yaw;
    public Rotation2d gyroOffset;

    public Pose2d botPose = new Pose2d();
}
