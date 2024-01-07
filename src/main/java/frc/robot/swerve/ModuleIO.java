package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ModuleIO {
    void updateInputs(SwerveModuleInputs inputs);

    Rotation2d getAngle();

    void setAngle(Rotation2d angle);

    double getVelocity();

    void setVelocity(double velocity);

    default SwerveModulePosition getModulePosition() {
        return null;
    }

    default void updateOffset(double offset) {}

    default void neutralOutput() {}

    default boolean encoderConnected() {
        return false;
    }

    default Command checkModule() {
        return Commands.none();
    }

    default void stop() {}
}
