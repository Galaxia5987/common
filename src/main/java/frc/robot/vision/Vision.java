package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import lib.Utils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final VisionModule[] modules;
    private final EstimatedRobotPose[] results;

    private Vision(VisionModule... modules) {
        this.modules = modules;
        results = new EstimatedRobotPose[modules.length];
    }

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(VisionModule... modules) {
        INSTANCE = new Vision(modules);
    }

    public EstimatedRobotPose[] getResults() {
        return results;
    }

    public double getAverageAmbiguity() {
        List<Double> ambiguityList =
                Arrays.stream(modules).map((module) -> module.inputs.averageAmbiguity).toList();
        return Utils.averageAmbiguity(ambiguityList);
    }

    @Override
    public void periodic() {
        List<Double> totalAverageAmbiguities = new ArrayList<>();
        double totalAverageAmbiguity;
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            for (VisionIO io : module.ios) {
                io.updateInputs(module.inputs);
                Logger.processInputs(io.getName(), module.inputs);
                results[i] = io.getLatestResult();
            }
            totalAverageAmbiguities.add(module.inputs.averageAmbiguity);
        }
        totalAverageAmbiguity = Utils.averageAmbiguity(totalAverageAmbiguities);
        Logger.recordOutput("totalAverageAmbiguity", totalAverageAmbiguity);
    }
}
