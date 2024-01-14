package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.XboxDrive;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionModule;
import frc.robot.vision.VisionSimIO;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive drive = SwerveDrive.getInstance();
    private final Vision vision =
            Vision.getInstance(
                    new VisionModule[] {
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "1"),
                                        new Transform3d(
                                                new Translation3d(
                                                        Math.sqrt(2) / 2, Math.sqrt(2) / 2, 0.5),
                                                new Rotation3d(0, 0, 45)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "2"),
                                        new Transform3d(
                                                new Translation3d(0, 1, 0.5),
                                                new Rotation3d(0, 0, 90)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "3"),
                                        new Transform3d(
                                                new Translation3d(
                                                        -Math.sqrt(2) / 2, Math.sqrt(2) / 2, 0.5),
                                                new Rotation3d(0, 0, 135)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "4"),
                                        new Transform3d(
                                                new Translation3d(-1, 0, 0.5),
                                                new Rotation3d(0, 0, 180)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "5"),
                                        new Transform3d(
                                                new Translation3d(
                                                        -Math.sqrt(2) / 2, -Math.sqrt(2) / 2, 0.5),
                                                new Rotation3d(0, 0, 225)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "6"),
                                        new Transform3d(
                                                new Translation3d(0, -1, 0.5),
                                                new Rotation3d(0, 0, 270)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "7"),
                                        new Transform3d(
                                                new Translation3d(
                                                        -Math.sqrt(2) / 2, -Math.sqrt(2) / 2, 0.5),
                                                new Rotation3d(0, 0, 315)))),
                        new VisionModule(
                                new VisionSimIO(
                                        new PhotonCamera(NetworkTableInstance.getDefault(), "8"),
                                        new Transform3d(
                                                new Translation3d(1, 0, 0.5),
                                                new Rotation3d(0, 0, 360))))
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
        return null;
    }
}
