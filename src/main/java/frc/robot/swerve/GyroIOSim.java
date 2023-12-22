package frc.robot.swerve;

import lib.math.differential.Integral;

public class GyroIOSim implements GyroIO {
    private final Integral yaw = new Integral();

    @Override
    public double getYaw() {
        return yaw.get();
    }

    @Override
    public void resetGyro(double angle) {
        yaw.override(angle);
    }

    @Override
    public void updateInputs(SwerveDriveInputs inputs) {
        yaw.update(inputs.currentSpeeds[2]);
    }
}
