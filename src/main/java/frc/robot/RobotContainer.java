package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.XboxDrive;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionModule;
import frc.robot.vision.VisionSimIO;
import frc.robot.vision.heatMap.commands.TestCameras;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive drive = SwerveDrive.getInstance();
    private TestCameras testCameras =
            new TestCameras(
                    new VisionModule(
                            new VisionSimIO(
                                    new PhotonCamera(NetworkTableInstance.getDefault(), "camera1"),
                                    VisionConstants.ROBOT_TO_CAM[0])));
    private final Vision vision =
            Vision.getInstance(
                    new VisionModule[] {
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(
                                                NetworkTableInstance.getDefault(), "camera1"),
                                        VisionConstants.ROBOT_TO_CAM[0]))
                    });
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

    private final JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton rightJoystickTrigger = new JoystickButton(rightJoystick, 1);

    private final JoystickButton a =
            new JoystickButton(xboxController, XboxController.Button.kA.value);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(new XboxDrive(drive, xboxController));
    }

    private void configureButtonBindings() {
        leftJoystickTrigger.onTrue(new InstantCommand(drive::resetGyro));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return testCameras;
    }
}
