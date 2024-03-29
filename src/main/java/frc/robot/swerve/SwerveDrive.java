package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import lib.PoseEstimation;
import lib.controllers.DieterController;
import lib.math.differential.Derivative;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    public static final Lock odometryLock = new ReentrantLock();
    private static SwerveDrive INSTANCE = null;
    private final SwerveModule[] modules; // FL, FR, RL, RR

    @AutoLogOutput
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final List<SwerveModulePosition[]> highFreqModulePositions = new ArrayList<>(Collections.singleton(modulePositions));

    private final GyroIO gyro;
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    SwerveConstants.WHEEL_POSITIONS[0],
                    SwerveConstants.WHEEL_POSITIONS[1],
                    SwerveConstants.WHEEL_POSITIONS[2],
                    SwerveConstants.WHEEL_POSITIONS[3]);
    private final Derivative acceleration = new Derivative();
    private final LinearFilter accelFilter = LinearFilter.movingAverage(15);
    private final PoseEstimation poseEstimator;
    private final SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();
    @AutoLogOutput private Pose2d botPose = new Pose2d();

    private SwerveDrive(GyroIO gyroIO, double[] wheelOffsets, ModuleIO... moduleIOs) {
        this.gyro = gyroIO;
        modules = new SwerveModule[moduleIOs.length];
        for (int i = 0; i < moduleIOs.length; i++) {
            modules[i] = new SwerveModule(moduleIOs[i], i + 1, wheelOffsets[i]);
        }

        updateModulePositions();

        poseEstimator = new PoseEstimation(this);
    }

    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    public static void initialize(GyroIO gyroIO, double[] offsets, ModuleIO... moduleIOs) {
        INSTANCE = new SwerveDrive(gyroIO, offsets,  moduleIOs);
    }

    /**
     * Updates the offset for the gyro.
     *
     * @param angle The desired angle. [rad]
     */
    public void resetGyro(double angle) {
        Rotation2d angleR2d = new Rotation2d(angle);
        gyro.resetGyro(angleR2d);
    }

    public void resetGyro() {
        resetGyro(0);
    }

    /**
     * Gets the raw yaw reading from the gyro.
     *
     * @return Yaw angle reading from gyro. [rad]
     */
    public Rotation2d getRawYaw() {
        return loggerInputs.rawYaw;
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     *
     * @return Yaw angle with offset. [rad]
     */
    public Rotation2d getYaw() {
        return loggerInputs.yaw;
    }

    public Rotation2d getPitch() {
        return loggerInputs.pitch;
    }

    /**
     * Sets the module states to the desired module states.
     *
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        loggerInputs.desiredModuleStates = desiredModuleStates;
    }

    public void updateModulePositions() {
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getModulePosition();
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public List<SwerveModulePosition[]> getHighFreqModulePositions() {
        return highFreqModulePositions;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getVelocity() {
        return loggerInputs.linearVelocity;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return loggerInputs.currentSpeeds;
    }

    public double[] getHighFreqTimeStamps() {
        return modules[0].getHighFreqTimestamps();
    }

    public void updateHighFreqPose() {
        highFreqModulePositions.clear();
        int sampleCount = Integer.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            sampleCount = Math.min(sampleCount, modules[i].getHighFreqAngles().length);
            sampleCount = Math.min(sampleCount, modules[i].getHighFreqDriveDistances().length);
            sampleCount = Math.min(sampleCount, modules[i].getHighFreqTimestamps().length);
        }
        for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
            // Read wheel positions
            SwerveModulePosition[] tempHighFreqModulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                tempHighFreqModulePositions[moduleIndex] =
                        new SwerveModulePosition(
                                modules[moduleIndex].getHighFreqDriveDistances()[sampleIndex],
                                Rotation2d.fromRadians(
                                        modules[moduleIndex].getHighFreqAngles()[sampleIndex]));
                highFreqModulePositions.add(tempHighFreqModulePositions);
            }
            poseEstimator.updatePose();
            botPose = poseEstimator.getEstimatedPose();
        }
    }

    public Pose2d getBotPose() {
        return botPose;
    }

    public void resetPose(Pose2d pose) {
        botPose = pose;
        poseEstimator.resetPose(pose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public Command checkSwerve() {
        var command =
                Stream.of(modules)
                        .map(SwerveModule::checkModule)
                        .reduce(Commands.none(), Commands::parallel)
                        .withName("Check Swerve");
        command.addRequirements(this);
        return command;
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleState(new SwerveModuleState(0, modules[i].getModuleState().angle));
        }
    }

    public void lock() {
        modules[0].setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        modules[1].setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        modules[2].setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
        modules[3].setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
    }

    public void updateOffsets(double[] offsets) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateOffset(new Rotation2d(Units.rotationsToRadians(offsets[i])));
        }
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = chassisSpeeds;

        ChassisSpeeds fieldOrientedChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond,
                        getYaw());

        if (new ChassisSpeeds(0, 0, 0).equals(chassisSpeeds)) {
            Arrays.stream(modules).forEach(SwerveModule::stop);
            return;
        }

        if (fieldOriented) {
            chassisSpeeds = fieldOrientedChassisSpeeds;
        }
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the frc.robot.swerve
     *
     * @param xOutput percentage of the max possible x speed
     * @param yOutput percentage of the max possible the y speed
     * @param omegaOutput percentage of the max possible rotation speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds =
                new ChassisSpeeds(
                        SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                        SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                        SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput);

        drive(chassisSpeeds, fieldOriented);
    }

    public Command driveCommand(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotation,
            double deadband,
            BooleanSupplier fieldOriented) {
        return run(
                () ->
                        drive(
                                MathUtil.applyDeadband(forward.getAsDouble(), deadband),
                                MathUtil.applyDeadband(strafe.getAsDouble(), deadband),
                                MathUtil.applyDeadband(rotation.getAsDouble(), deadband),
                                fieldOriented.getAsBoolean()));
    }

    public Command turnCommand(double rotation, double turnTolerance) {
        PIDController turnController =
                new DieterController(
                        SwerveConstants.ROTATION_KP.get(),
                        SwerveConstants.ROTATION_KI.get(),
                        SwerveConstants.ROTATION_KD.get(),
                        SwerveConstants.ROTATION_KDIETER.get());
        turnController.setTolerance(turnTolerance);
        return run(() ->
                        drive(
                                0,
                                0,
                                turnController.calculate(getYaw().getRotations(), rotation),
                                false))
                .until(turnController::atSetpoint);
    }

    public void updateSwerveInputs() {
        for (int i = 0; i < modules.length; i++) {
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
            loggerInputs.currentModuleStates[i] = modules[i].getModuleState();
        }

        loggerInputs.currentSpeeds =
                kinematics.toChassisSpeeds(
                        loggerInputs.currentModuleStates[0],
                        loggerInputs.currentModuleStates[1],
                        loggerInputs.currentModuleStates[2],
                        loggerInputs.currentModuleStates[3]);

        loggerInputs.linearVelocity =
                Math.hypot(
                        loggerInputs.currentSpeeds.vxMetersPerSecond,
                        loggerInputs.currentSpeeds.vyMetersPerSecond);

        acceleration.update(loggerInputs.linearVelocity);
        loggerInputs.acceleration = accelFilter.calculate(acceleration.get());

        loggerInputs.supplyCurrent =
                Arrays.stream(modules).mapToDouble(SwerveModule::getSupplyCurrent).sum();

        loggerInputs.statorCurrent =
                Arrays.stream(modules).mapToDouble(SwerveModule::getStatorCurrent).sum();
    }

    public void updateGyroInputs() {
        loggerInputs.rawYaw = gyro.getRawYaw();
        loggerInputs.yaw = gyro.getYaw();
        loggerInputs.pitch = gyro.getPitch();
        gyro.updateInputs(loggerInputs);
    }

    public void periodic() {
        odometryLock.lock();
        for (SwerveModule module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        updateHighFreqPose();

        updateSwerveInputs();

        updateModulePositions();

        updateGyroInputs();

        SwerveDriveKinematics.desaturateWheelSpeeds(
                loggerInputs.desiredModuleStates, SwerveConstants.MAX_X_Y_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(loggerInputs.desiredModuleStates[i]);
        }

        Logger.processInputs("SwerveDrive", loggerInputs);
    }
}
