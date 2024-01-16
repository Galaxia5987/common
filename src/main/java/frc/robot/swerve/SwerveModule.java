package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.math.differential.BooleanTrigger;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleInputsAutoLogged loggerInputs = new SwerveModuleInputsAutoLogged();

    private final ModuleIO io;

    private final int number;
    private final BooleanTrigger encoderTrigger = new BooleanTrigger();
    private final Timer timer = new Timer();

    public SwerveModule(ModuleIO io, int number) {
        this.io = io;
        this.number = number;

        timer.start();
        timer.reset();
    }

    /**
     * Gets the state of a module.
     *
     * @return The state of a module.
     */
    public SwerveModuleState getModuleState() {
        return io.getModuleState();
    }

    /**
     * Sets the module to a desired module state.
     *
     * @param moduleState A module state to set the module to.
     */
    public void setModuleState(SwerveModuleState moduleState) {
        moduleState = SwerveModuleState.optimize(moduleState, loggerInputs.angle);
        io.setVelocity(moduleState.speedMetersPerSecond);
        io.setAngle(moduleState.angle);
    }

    /**
     * Gets the position of the module.
     *
     * @return Position of the module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(loggerInputs.moduleDistance, loggerInputs.angle);
    }

    /**
     * Gets the stator current of both motors combined.
     *
     * @return Sum of the drive motor stator current and angle motor stator current. [amps]
     */
    public double getStatorCurrent() {
        return loggerInputs.driveMotorStatorCurrent + loggerInputs.angleMotorStatorCurrent;
    }

    /**
     * Gets the supply current of both motors combined.
     *
     * @return Sum of the drive motor supply current and angle motor supply current. [amps]
     */
    public double getSupplyCurrent() {
        return loggerInputs.driveMotorSupplyCurrent + loggerInputs.angleMotorSupplyCurrent;
    }

    /**
     * Gets the position of the absolute encoder.
     *
     * @return Position of the absolute encoder. [sensor ticks]
     */
    public double getPosition() {
        return loggerInputs.absolutePosition;
    }

    /**
     * Updates the position of the angle motor with an offset and an absolute encoder.
     *
     * @param offset The offset to update the angle motor's position. [sensor ticks]
     */
    public void updateOffset(double offset) {
        io.updateOffset(offset);
    }

    public void stopMotor() {
        io.stop();
    }

    public boolean encoderConnected() {
        return io.encoderConnected();
    }

    public Command checkModule() {
        return io.checkModule().withName("CheckModule_" + number);
    }

    @Override
    public void periodic() {
        io.updateInputs(loggerInputs);

        Logger.processInputs("module_" + number, loggerInputs);

        encoderTrigger.update(io.encoderConnected());

        if (timer.hasElapsed(1)) {
            io.updateOffset(SwerveConstantsNeo.OFFSETS[number - 1]);
            timer.reset();
        }
    }
}
