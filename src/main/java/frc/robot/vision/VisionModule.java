package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Arrays;

public class VisionModule {
    public final VisionIO[] ios;
    public final VisionInputsAutoLogged inputs;
    public final Transform3d[] robotToCams;

    public VisionModule(VisionIO... ios) {
        this.ios = ios;
        this.inputs = new VisionInputsAutoLogged();
        robotToCams =
                Arrays.stream(ios)
                        .map(VisionIO::getCameraToRobot)
                        .toList()
                        .toArray(new Transform3d[0]);
    }
}
