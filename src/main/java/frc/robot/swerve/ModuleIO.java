package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.webconstants.LoggedTunableNumber;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    Rotation2d getAngle();

    void setAngle(Rotation2d angle);

    double getVelocity();

    void setVelocity(double velocity);

    void setAngleVelocity(double velocity);

    SwerveModuleState getModuleState();

    SwerveModulePosition getModulePosition();

    void stop();

    void updateSlot0Configs();

    boolean hasPIDChanged(LoggedTunableNumber[] PIDValues);

    default void updateOffset(Rotation2d offset) {}

    default boolean encoderConnected() {
        return true;
    }
}
