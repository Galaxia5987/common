package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import lib.Utils;
import org.littletonrobotics.junction.Logger;

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
            INSTANCE = new Vision(modules);
        }
        return INSTANCE;
    }

    public Result[] getResults() {
        return results;
    }

    public double getAverageAmbiguity() {
        ArrayList<Double> ambiguityList = new ArrayList<>();
        for (VisionModule module : modules) {
            ambiguityList.add(module.inputs.targetAmbiguity);
        }
        return Utils.averageAmbiguity(ambiguityList);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.processInputs(module.io.getName(), module.inputs);
            results[i] = module.io.getLatestResult();
        }
    }
}
