package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;
import org.littletonrobotics.junction.Logger;

import static frc.robot.vision.VisionConstants.ROBOT_TO_CAM;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final VisionModule[] modules;
    private final Result[] results;

    private Vision(VisionModule... modules) {
        this.modules = modules;
        results = new Result[modules.length];
    }

    public static Vision getInstance(VisionModule[] modules) {
        if (INSTANCE == null) {
            INSTANCE = new Vision(
                modules
            );
        }
        return INSTANCE;
    }

    public Result[] getResults() {
        return results;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.getInstance().processInputs(module.name, module.inputs);
            results[i] = module.io.getLatestResult();
            Logger.getInstance().recordOutput("RobotToCamLeft", new Pose3d(SwerveDrive.getInstance().getBotPose()).plus(ROBOT_TO_CAM[0]));
        }
    }
}
