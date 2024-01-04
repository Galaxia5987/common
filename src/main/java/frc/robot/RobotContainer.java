package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.conveyor.Conveyor;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.Deploy;
import frc.robot.intake.commands.Retract;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.XboxDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Conveyor conveyor = Conveyor.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
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
        swerveDrive.setDefaultCommand(
                new XboxDrive(swerveDrive, xboxController)
        );
    }

    private void configureButtonBindings() {
        a.onTrue(new InstantCommand(swerveDrive::resetGyro));
        rb.onTrue(new Deploy(0.8, 0.3)
                .alongWith(conveyor.spin(0.4)));
        rb.onFalse(new Retract(0.8, -0.4)
                .andThen(Commands.parallel(
                                intake.spin(0),
                                conveyor.spin(0)
                        )
                )
        );
        lb.onTrue(new Deploy(-0.8, 0.3)
                .alongWith(conveyor.spin(-0.4)));
        lb.onFalse(new Retract(-0.8, -0.4)
                .andThen(Commands.parallel(
                                intake.spin(0),
                                conveyor.spin(0)
                        )
                )
        );
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
