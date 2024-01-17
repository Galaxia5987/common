package frc.robot.vision.heatMap.commands;

import com.opencsv.CSVWriter;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.vision.SimVisionSystem;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionModule;
import frc.robot.vision.heatMap.HeatMap;
import frc.robot.vision.heatMap.HeatMapConstants;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.IntStream;
import lib.Utils;
import org.littletonrobotics.junction.Logger;

public class TestCameras extends Command {
    private final SimVisionSystem visionSim = SimVisionSystem.getInstance();

    private final VisionModule[] visionModules;
    private final double[] heightArr = DoubleStream.iterate(HeatMapConstants.heightMinimumRange, n -> n + HeatMapConstants.heightJumps)
            .limit((long) Math.ceil((HeatMapConstants.heightMaximum - HeatMapConstants.heightMinimumRange) / HeatMapConstants.heightJumps) + 1)
            .toArray();
    private final Pair[] gridsToCheck =
            IntStream.rangeClosed(1, (int) (HeatMapConstants.xLength / HeatMapConstants.squareLength))
                    .boxed()
                    .flatMap(i -> IntStream.rangeClosed(1, (int) (HeatMapConstants.yLength / HeatMapConstants.squareLength)).mapToObj(j -> new Pair<>(i, j)))
                    .toArray(Pair[]::new);
    private final double[] angleArr = IntStream.rangeClosed(0, 360)
            .filter(i -> i % HeatMapConstants.robotAngleJumps == 0)
            .mapToDouble(Math::toRadians)
            .toArray();


    public TestCameras(Vision vision) {
        this.visionModules = vision.getVisionModules();
    }

    // TODO: change this to an array for each param(5 arrays)
    //  and then every execute iterate over a combination and in the next one move to the next combination
    //  you need 5 boolean flags that says when something is finished
    @Override
    public void execute() {
        double estimatedModuleTime =
                HeatMapConstants.iterationTime
                        * (HeatMapConstants.heightMaximum - HeatMapConstants.heightMinimumRange) / HeatMapConstants.heightJumps
                        * gridsToCheck.length
                        * (360 / HeatMapConstants.robotAngleJumps);
        double totalEstimatedTime = estimatedModuleTime * visionModules.length;
        Logger.recordOutput("TotalEstimatedTime", totalEstimatedTime);

        for (VisionModule visionModule : visionModules) {
            heatMap = HeatMap.getInstance(visionModule);
            Logger.recordOutput("EstimatedModuleTime", estimatedModuleTime);
            for (int height = HeatMapConstants.heightMinimumRange;
                    height < HeatMapConstants.heightMaximum;
                    height += HeatMapConstants.heightJumps) {
                double optimalPitch = Utils.calcPitchByHeight(height);
                visionSim.adjustCameraPose(visionModule, height, optimalPitch);
                heatMap.resetHeatMap();

                for (Pair grid : gridsToCheck) {
                    for (int angle = 0; angle < 360; angle += HeatMapConstants.robotAngleJumps) {
                        robotPose = heatMap.gridToPose(grid, optimalPitch, height);
                        heatMap.update(robotPose);
                    }
                }

                // save heatMap to .csv file
                String csvFilePath = "HeatMap_" + height + "_" + optimalPitch;
                try (CSVWriter csvWriter = new CSVWriter(new FileWriter(csvFilePath))) {
                    Arrays.stream(heatMap.getFieldArr())
                            .map(
                                    row ->
                                            Arrays.stream(row)
                                                    .mapToObj(String::valueOf)
                                                    .toArray(String[]::new))
                            .forEach(csvWriter::writeNext);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                System.out.println("CSV file has been created successfully.");
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
