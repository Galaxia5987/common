package frc.robot.vision.heatMap.commands;

import com.opencsv.CSVWriter;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import java.util.stream.DoubleStream;
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

    private int visionModuleIndex = 0;
    private int heightIndex = 0;
    private int gridsToCheckIndex = 0;
    private int angleArrIndex = 0;

    public TestCameras(Vision vision) {
        this.visionModules = vision.getVisionModules();
        double totalEstimatedTime =
                HeatMapConstants.iterationTime
                        * (HeatMapConstants.heightMaximum - HeatMapConstants.heightMinimumRange)
                        / HeatMapConstants.heightJumps
                        * gridsToCheck.length
                        * (2 * Math.PI / Math.toRadians(HeatMapConstants.robotAngleJumps))
                        * visionModules.length;
        Logger.recordOutput("TotalEstimatedTime", totalEstimatedTime);
    }

    @Override
    public void initialize() {
        visionSim.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        visionSim.setUseSwerve(false);
    }

    @Override
    public void execute() {
        double optimalPitch = Utils.calcPitchByHeight(heightArr[heightIndex]);

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
        HeatMap heatMap = HeatMap.getInstance(visionModules[visionModuleIndex]);
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
