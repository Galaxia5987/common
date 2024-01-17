package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(VisionIO... ios) {
        INSTANCE = new Vision(
                Arrays.stream(ios).map(
                        VisionModule::new
                ).toList().toArray(new VisionModule[0])
        );
    }

    public Result[] getResults() {
        return results;
    }

    public double getAverageAmbiguity() {
        ArrayList<Double> ambiguityList = new ArrayList<>();
        for (VisionModule module : modules) {
            ambiguityList.add(module.inputs.bestTargetAmbiguity);
        }
        return Utils.averageAmbiguity(ambiguityList);
    }

    @Override
    public void periodic() {
        List<Double> totalAverageAmbiguties = new ArrayList<>();
        double totalAverageAMBIGUITY;
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.processInputs(module.io.getName(), module.inputs);
            results[i] = module.io.getLatestResult();
            totalAverageAmbiguties.add(module.inputs.averageAmbiguity);
        }
        totalAverageAMBIGUITY = Utils.averageAmbiguity(totalAverageAmbiguties);
        Logger.recordOutput("totalAverageAmbiguties", totalAverageAMBIGUITY);
    }
}
