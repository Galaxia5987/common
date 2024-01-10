package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
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
            ambiguityList.add(module.inputs.bestTargetAmbiguity);
        }
        return Utils.averageAmbiguity(ambiguityList);
    }

    @Override
    public void periodic() {
        List<Double> totalAvaregeAmbiguties = new ArrayList<>();
        double avaregeAmbiguties;
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.processInputs(module.io.getName(), module.inputs);
            results[i] = module.io.getLatestResult();
            totalAvaregeAmbiguties.add(module.inputs.averageAmbiguity);
            avaregeAmbiguties =  Utils.averageAmbiguity(totalAvaregeAmbiguties);
            System.out.println(avaregeAmbiguties);
        }
        avaregeAmbiguties = 0;
    }
    /
}
