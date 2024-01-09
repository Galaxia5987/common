package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class VisionModule {

    public final String name;
    public final VisionIO io;
    public final VisionInputsAutoLogged inputs;
    public final Transform3d ROBOT_TO_CAM;

    public VisionModule(VisionIO io) {
        this.io = io;
        this.inputs = new VisionInputsAutoLogged();
        this.ROBOT_TO_CAM = ROBOT_TO_CAM;
    }
}
