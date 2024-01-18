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
    /**
     * Array for the camera heights to check during camera simulation
     * Based on the Heatmap Constants
     * Units are in meters
     */
    private final double[] heightArr =
            DoubleStream.iterate(
                            HeatMapConstants.heightMinimumRange,
                            n -> n + HeatMapConstants.heightDelta)
                    .limit(
                            (long)
                                            Math.ceil(
                                                    (HeatMapConstants.heightMaximumRange
                                                                    - HeatMapConstants
                                                                            .heightMinimumRange)
                                                            / HeatMapConstants.heightDelta)
                                    + 1)
                    .toArray();
    /**
     * 2d Array of Pair Integers to check during camera simulation
     * Based on Heatmap Constants
     */
    //TODO: change grid size and field size to match navgrid.json
    // find a way to read navgrid.json into gridsToCheck array
    private final Pair[] gridsToCheck =
            IntStream.rangeClosed(
                            1, (int) (HeatMapConstants.xLength / HeatMapConstants.squareLength))
                    .boxed()
                    .flatMap(
                            i ->
                                    IntStream.rangeClosed(
                                                    1,
                                                    (int)
                                                            (HeatMapConstants.yLength
                                                                    / HeatMapConstants
                                                                            .squareLength))
                                            .mapToObj(j -> new Pair<>(i, j)))
                    .toArray(Pair[]::new);
    /**
     * Array of angels to go through in each grid
     * Based on Heatmap Constants
     * Units are in rads
     */
    private final double[] angleArr =
            IntStream.rangeClosed(0, 360)
                    .filter(i -> i % HeatMapConstants.robotAngleDelta == 0)
                    .mapToDouble(Math::toRadians)
                    .toArray();

    /**
     * Index counter for the visionModule Array
     */
    private int visionModuleIndex = 0;
    /**
     * Index counter for the height Array
     */
    private int heightIndex = 0;
    /**
     * Index counter for the gridsToCheck Array
     */
    private int gridsToCheckIndex = 0;
    /**
     * Index counter for the angle Array
     */
    private int angleArrIndex = 0;

    /**
     * Constructor for the TestCameras command
     * calculates estimated runtime and logs it
     *
     * @param vision Instance of the vision class
     */
    public TestCameras(Vision vision) {
        this.visionModules = vision.getVisionModules();
        double totalEstimatedTime =
                HeatMapConstants.iterationTime
                        * (HeatMapConstants.heightMaximumRange - HeatMapConstants.heightMinimumRange)
                        / HeatMapConstants.heightDelta
                        * gridsToCheck.length
                        * (2 * Math.PI / Math.toRadians(HeatMapConstants.robotAngleDelta))
                        * visionModules.length;
        Logger.recordOutput("TotalEstimatedTime", totalEstimatedTime);
    }

    /**
     * Init for the TestCameras command
     * sets the robotPose in visionSim, so it won't be null
     * sets the visionSim class to not use swerve Positions
     */
    @Override
    public void initialize() {
        visionSim.setRobotPose(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        visionSim.setUseSwerve(false);
    }

    /**
     * Execute function for the TestCameras command
     * needs to be called repeatedly
     *
     * Works by calculating the optimal pitch for the current height which we get using the heightArr and the heightIndex
     * then adjusts the camera pose, calculates the robot pose based on the grid sets it to the SimVisionSystem and updates it in the heatmap class
     *
     * angleArrIndex needs to be increased in every iteration so the robot will check all angles
     * According to the angelArrIndex it updates the gridIndex after it checks all the grids in all angles
     * it will write the 2d fieldArr with averageAmbiguities values to a .csv file in the heatMapsCSVs with the height values that was used the visionModule that was used
     * after it checked all the possible height in the vision module it will move on to the next VisionModule if there is one
     */
    @Override
    public void execute() {
        double optimalPitch = Utils.calcPitchByHeight(heightArr[heightIndex]);

        HeatMap heatMap = HeatMap.getInstance(visionModules[visionModuleIndex]);
        visionSim.adjustCameraPose(
                visionModules[visionModuleIndex], heightArr[heightIndex], optimalPitch);
        Pose3d robotPose =
                heatMap.gridToPose(
                        gridsToCheck[gridsToCheckIndex],
                        optimalPitch,
                        heightArr[heightIndex],
                        angleArr[angleArrIndex]);
        visionSim.setRobotPose(robotPose.toPose2d());
        heatMap.update(robotPose);
        Logger.recordOutput("TestCamerasBotPose", robotPose.toPose2d());

        if (angleArr.length == angleArrIndex + 1) {
            angleArrIndex = 0;
            gridsToCheckIndex++;
        }
        if (gridsToCheck.length == gridsToCheckIndex + 1) {
            gridsToCheckIndex = 0;
            heightIndex++;

            // save heatMap to .csv file
            String csvFilePath = "C:\\Users\\rakra\\Common\\src\\main\\java\\frc\\robot\\vision\\heatMap\\heatMapsCSVs\\HeatMap_"
                    + heightArr[heightIndex]
                    + "__"
                    + visionModuleIndex
                    + ".csv";

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
            heatMap.resetHeatMap();
        }
        if (heightArr.length == heightIndex + 1) {
            heightIndex = 0;
            if (visionModules.length != visionModuleIndex + 1) {
                visionModuleIndex++;
            }
        }
        angleArrIndex++;
    }
}
