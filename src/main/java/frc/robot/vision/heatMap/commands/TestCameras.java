package frc.robot.vision.heatMap.commands;

import com.opencsv.CSVWriter;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.SimVisionSystem;
import frc.robot.vision.VisionModule;
import frc.robot.vision.heatMap.HeatMap;
import frc.robot.vision.heatMap.HeatMapConstants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.IntStream;
import lib.Utils;
import org.littletonrobotics.junction.Logger;

public class TestCameras extends Command {
    private final SimVisionSystem visionSim = SimVisionSystem.getInstance();
    private VisionModule[] visionModules;
    private HeatMap heatMap;
    private HeatMapField heatMapField;

    private Pose3d robotPose;

    private final Pair[] gridsToCheck =
            IntStream.rangeClosed(1, 8)
                    .boxed()
                    .flatMap(i -> IntStream.rangeClosed(1, 4).mapToObj(j -> new Pair<>(i, j)))
                    .toArray(Pair[]::new);

    public TestCameras(VisionModule... visionModules) {
        this.visionModules = visionModules;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        for (VisionModule visionModule: visionModules) {
            heatMap = HeatMap.getInstance(visionModule);
            heatMapField = new HeatMapField(heatMap.getFieldArr());
            Logger.recordOutput("EstimatedModuleTime", estimatedModuleTime);
            for (int height = HeatMapConstants.heightMinimumRange;
                    height < HeatMapConstants.heightMaximum;
                    height += HeatMapConstants.heightJumps) {
                double optimalPitch = Utils.calcPitchByHeight(height);
                visionSim.adjustCameraPose(visionModule, height, optimalPitch);

                for (Pair grid : gridsToCheck) {
                    for (int angle = 0; angle < 360; angle += HeatMapConstants.robotAngleJumps) {
                        robotPose = heatMap.gridToPose(grid, optimalPitch, height);
                        heatMap.update(robotPose);
                    }
                }
                heatMapField.setFieldArr(heatMap.getFieldArr());

                // save heatMap to .csv file
                String csvFilePath = "HeatMap_" + height + "_" + optimalPitch;
                try (CSVWriter csvWriter = new CSVWriter(new FileWriter(csvFilePath))) {
                    Arrays.stream(heatMapField.getFieldArr())
                            .map(
                                    row ->
                                            Arrays.stream(row)
                                                    .mapToObj(String::valueOf)
                                                    .toArray(String[]::new))
                            .forEach(csvWriter::writeNext);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return super.getRequirements();
    }
}
