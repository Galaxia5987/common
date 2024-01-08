package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

public class VisionModule {

    public final String name;
    public final VisionIO io;
    public final VisionInputsAutoLogged inputs;
    public final Transform3d ROBOT_TO_CAM;

    public VisionModule(String name, VisionIO io, Transform3d ROBOT_TO_CAM) {
        this.name = name;
        this.io = io;
        this.inputs = new VisionInputsAutoLogged();
        this.ROBOT_TO_CAM = ROBOT_TO_CAM;
    }

    public static VisionModule photonVisionIO(String name, int index) {
        return new VisionModule(
                name,
                new PhotonVisionIO(new PhotonCamera(name), VisionConstants.ROBOT_TO_CAM[index])
        );
    }
    public static VisionModule simIO(String name,  int index){
        return new VisionModule(
                name,
                new VisionSimIO(VisionConstants.ROBOT_TO_CAM[index])
        );
    }
}
