package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionModule {
    public final VisionIO io;
    public final VisionInputsAutoLogged inputs;
    public final Transform3d robotToCam;

    public VisionModule(VisionIO io) {
        this.io = io;
        this.inputs = new VisionInputsAutoLogged();
        this.robotToCam = io.getCameraToRobot();
    }
}
