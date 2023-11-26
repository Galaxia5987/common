package frc.robot.common.swerve;

public interface GyroIO {
    void updateInputs(SwerveDriveInputs inputs);

    double getYaw();

    default double getRawYaw() {
        return ;
    }

    default double getPitch() {
        return 0;
    }

    void resetGyro(double angle);
}
