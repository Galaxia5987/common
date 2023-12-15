package frc.robot.swerve;

public interface GyroIO {
    void updateInputs(SwerveDriveInputs inputs);

    double getYaw();

    default double getRawYaw() {
        return 0;
    }

    default double getPitch() {
        return 0;
    }

    void resetGyro(double angle);
}
