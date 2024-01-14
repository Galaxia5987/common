package frc.robot.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.math.differential.Derivative;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive INSTANCE = null;
    private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, RL, RR
    @AutoLogOutput
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private final GyroIO gyro;
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    SwerveConstants.WHEEL_POSITIONS[0],
                    SwerveConstants.WHEEL_POSITIONS[1],
                    SwerveConstants.WHEEL_POSITIONS[2],
                    SwerveConstants.WHEEL_POSITIONS[3]);

    private final Derivative acceleration = new Derivative();
    private final LinearFilter accelFilter = LinearFilter.movingAverage(15);

    public static final Lock odometryLock = new ReentrantLock();
    @AutoLogOutput
    private Pose2d botPose = new Pose2d();
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();

    private SwerveDrive(
            boolean isReal,
            boolean isNeo,
            boolean[] drivesInverted,
            boolean[] anglesInverted,
            int[] driveIds,
            int[] angleIds,
            int[] encoderIds) {
        if (isReal) {
            for (int i = 0; i < modules.length; i++) {
                ModuleIO io;
                if (!isNeo) {
                    io =
                            new ModuleIOReal(
                                    driveIds[i],
                                    angleIds[i],
                                    encoderIds[i],
                                    SwerveConstants.DRIVE_MOTOR_CONFIGS,
                                    SwerveConstants.ANGLE_MOTOR_CONFIGS);
                } else {
                    io =
                            new ModuleIOReal(
                                    driveIds[i],
                                    angleIds[i],
                                    encoderIds[i],
                                    SwerveConstants.DRIVE_MOTOR_CONFIGS,
                                    SwerveConstants.ANGLE_MOTOR_CONFIGS);
                    ;
                }

                modules[i] = new SwerveModule(io, i + 1);
            }

            gyro = new GyroIOReal();
        } else {
            for (int i = 0; i < modules.length; i++) {
                ModuleIO io = new ModuleIOSim();
                modules[i] = new SwerveModule(io, i + 1);
            }

            gyro = new GyroIOSim();
        }
        updateModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRawYaw(), modulePositions, botPose);
    }

    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    public static void setInstance(
            boolean isReal,
            boolean isNeo,
            boolean[] drivesInverted,
            boolean[] anglesInverted,
            int[] driveIds,
            int[] angleIds,
            int[] encoderIds) {
        if (INSTANCE == null) {
            INSTANCE =
                    new SwerveDrive(
                            isReal,
                            isNeo,
                            drivesInverted,
                            anglesInverted,
                            driveIds,
                            angleIds,
                            encoderIds);
        }
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

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getVelocity() {
        return loggerInputs.linearVelocity;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return loggerInputs.currentSpeeds;
    }

    public void updateHighFreqPose() {
        int deltaCount = Integer.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            deltaCount = Math.min(deltaCount, modules[i].getHighFreqAngles().length);
            deltaCount = Math.min(deltaCount, modules[i].getHighFreqDriveDistances().length);
        }
        for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
            // Read wheel positions
            SwerveModulePosition[] highFreqModulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                highFreqModulePositions[moduleIndex] = new SwerveModulePosition(
                        modules[moduleIndex].getHighFreqDriveDistances()[deltaIndex],
                        Rotation2d.fromRadians(modules[moduleIndex].getHighFreqAngles()[deltaIndex])
                );
            }
            poseEstimator.update(getRawYaw(), highFreqModulePositions);
            botPose = poseEstimator.getEstimatedPosition();
        }
    }

    public Pose2d getBotPose() {
        return botPose;
    }

    public void resetPose(Pose2d pose) {
        botPose = pose;
        poseEstimator.resetPosition(getRawYaw(), modulePositions, pose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public boolean encodersConnected() {
        boolean connected = true;
        for (int i = 0; i < 4; i++) {
            connected &= modules[i].encoderConnected();
        }
        return connected;
    }

    public Command checkSwerve() {
        var command = Arrays.stream(modules).map((module) -> Commands.run(module::checkModule))
                .reduce(Commands.none(), Commands::parallel).withName("checkSwerve");
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
            for (SwerveModule module : modules) {
                module.stop();
            }
        }

        if (fieldOriented) {
            setModuleStates(kinematics.toSwerveModuleStates(fieldOrientedChassisSpeeds));
        } else {
            setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
        }
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the frc.robot.swerve
     *
     * @param xOutput     percentage of the x speed
     * @param yOutput     percentage of the y speed
     * @param omegaOutput percentage of the omega speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput, boolean fieldOriented) {
        ChassisSpeeds chassisSpeeds =
                new ChassisSpeeds(
                        SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                        SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                        SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput); // removed angleFF

        drive(chassisSpeeds, fieldOriented);
    }

    public void periodic() {
        odometryLock.lock();
        for (SwerveModule module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        updateHighFreqPose();

        for (int i = 0; i < modules.length; i++) {
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
            loggerInputs.currentModuleStates[i] = modules[i].getModuleState();
        }

        for (int i = 0; i < 3; i++) {
            loggerInputs.currentSpeeds =
                    kinematics.toChassisSpeeds(
                            loggerInputs.currentModuleStates[0],
                            loggerInputs.currentModuleStates[1],
                            loggerInputs.currentModuleStates[2],
                            loggerInputs.currentModuleStates[3]);
        }

        updateModulePositions();

        loggerInputs.linearVelocity =
                Math.hypot(loggerInputs.currentSpeeds.vxMetersPerSecond, loggerInputs.currentSpeeds.vyMetersPerSecond);

        acceleration.update(loggerInputs.linearVelocity);
        loggerInputs.acceleration = accelFilter.calculate(acceleration.get());

        loggerInputs.supplyCurrent =
                modules[0].getSupplyCurrent()
                        + modules[1].getSupplyCurrent()
                        + modules[2].getSupplyCurrent()
                        + modules[3].getStatorCurrent();

        loggerInputs.statorCurrent =
                modules[0].getStatorCurrent()
                        + modules[1].getStatorCurrent()
                        + modules[2].getStatorCurrent()
                        + modules[3].getStatorCurrent();

        loggerInputs.rawYaw = gyro.getRawYaw();
        loggerInputs.yaw = gyro.getYaw();
        loggerInputs.pitch = gyro.getPitch();
        gyro.updateInputs(loggerInputs);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                loggerInputs.desiredModuleStates,
                SwerveConstants.MAX_X_Y_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(loggerInputs.desiredModuleStates[i]);
        }

        Logger.processInputs("SwerveDrive", loggerInputs);
    }
}
