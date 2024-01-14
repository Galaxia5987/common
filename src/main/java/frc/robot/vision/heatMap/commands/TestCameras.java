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
    private HeatMapField heatMapField = new HeatMapField(heatMap.getFieldArr());

    private int heightRange = 0;
    private int heightJumps = 0;
    private int pitchRange = 0;
    private int pitchJumps = 0;
    private int robotAngleJumps = 0;

    private Pose3d robotPose;

    private double robotAngels[] = {0, 0, 0, 0, 0, 0, 0, 0};
    private Pair<Integer, Integer> gridsToCheck[] = ;//TODO: define grids to check

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
            for (int height = -heightRange; height < heightRange; height += heightRange) {
                for (int pitch = 0; pitch < pitchRange; pitch += pitchJumps) {
                    for (Pair<Integer, Integer> grid: gridsToCheck) {
                        for (int angle = 0; angle < 360; angle += robotAngleJumps) {
                            robotPose = heatMap.gridToPose(grid, pitch, height);
                            heatMap.update(robotPose);
                        }
                    }
                    heatMapField.setFieldArr(heatMap.getFieldArr());
                    //save heatmap
                    Logger.recordOutput("heatMap " + height + " " + pitch, Arrays.stream(heatMapField.getFieldArr()).);
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
