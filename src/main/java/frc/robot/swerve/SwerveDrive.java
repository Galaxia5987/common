package frc.robot.swerve;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Utils;
import lib.math.differential.Derivative;
import org.littletonrobotics.junction.Logger;


public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive INSTANCE = null;
    private final GyroIO gyro;
    private final SwerveModule[] modules = new SwerveModule[4]; //FL, FR, RL, RR
    private final LinearFilter accelFilter = LinearFilter.movingAverage(15);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            SwerveConstants.wheelPositions[0],
            SwerveConstants.wheelPositions[1],
            SwerveConstants.wheelPositions[2],
            SwerveConstants.wheelPositions[3]
    );
    private final SwerveDriveOdometry odometry;
    private final Derivative acceleration = new Derivative();
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private final SwerveDriveInputsAutoLogged loggerInputs = new SwerveDriveInputsAutoLogged();
    private final SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    private SwerveDrive(boolean isReal,
                        int[] driveIds,
                        int[] angleIds,
                        int[] encoderIds) {
        if (isReal) {
            for (int i = 0; i < modules.length; i++) {
                ModuleIO io = new ModuleIOReal(
                        driveIds[i],
                        angleIds[i],
                        encoderIds[i],
                        SwerveConstants.motionMagicConfigs[i],
                        i + 1);

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
        odometry = new SwerveDriveOdometry(
                kinematics, new Rotation2d(getYaw()),
                modulePositions
        );
    }

    public static SwerveDrive getInstance() {
        return INSTANCE;
    }

    public static void setInstance(boolean isReal,
                                   int[] driveIds,
                                   int[] angleIds,
                                   int[] encoderIds) {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive(isReal, driveIds, angleIds, encoderIds);
        }
    }

    /**
     * Updates the offset for the gyro.
     *
     * @param angle The desired angle. [rad]
     */
    public void resetGyro(double angle) {
        gyro.resetGyro(angle);
    }

    public void resetGyro() {
        resetGyro(0);
    }

    /**
     * Gets the raw yaw reading from the gyro.
     *
     * @return Yaw angle reading from gyro. [rad]
     */
    public double getRawYaw() {
        return loggerInputs.rawYaw;
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     *
     * @return Yaw angle with offset. [rad]
     */
    public double getYaw() {
        return loggerInputs.yaw;
    }

    public double getPitch() {
        return loggerInputs.pitch;
    }

    /**
     * Sets the module states to the desired module states.
     *
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        loggerInputs.desiredModuleStates = Utils.swerveModuleStatesToArray(desiredModuleStates);
    }

    public void updateModulePositions() {
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getModulePosition();
        }
    }

    public Pose2d getBotPose() {
        return Utils.arrayToPose2d(loggerInputs.botPose);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getVelocity() {
        return loggerInputs.linearVelocity;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return Utils.arrayToChassisSpeeds(loggerInputs.currentSpeeds);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(getYaw()), modulePositions, pose);
    }

    public void resetPose() {
        odometry.resetPosition(new Rotation2d(getYaw()), modulePositions, new Pose2d());
    }

    public boolean encodersConnected() {
        boolean connected = true;
        for (int i = 0; i < 4; i++) {
            connected &= modules[i].encoderConnected();
        }
        return connected;
    }

    public void checkSwerve() {
        for (int i = 0; i < 4; i++) {
            modules[i].checkModule();
        }
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

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = Utils.chassisSpeedsToArray(chassisSpeeds);

        ChassisSpeeds fieldOrientedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                new Rotation2d(getYaw())
        );

        if (chassisSpeeds.equals(new ChassisSpeeds(0, 0, 0))) {
            for (SwerveModule module : modules) {
                module.neutralOutput();
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
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput); //removed angleFF

        drive(chassisSpeeds, fieldOriented);
    }

    public void periodic() {
        updateModulePositions();
        odometry.update(new Rotation2d(getYaw()), modulePositions);

        loggerInputs.botPose[0] = odometry.getPoseMeters().getX();
        loggerInputs.botPose[1] = odometry.getPoseMeters().getY();
        loggerInputs.botPose[2] = odometry.getPoseMeters().getRotation().getRadians();

        for (int i = 0; i < modules.length; i++) {
            currentModuleStates[i] = modules[i].getModuleState();
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
        }

        Logger.recordOutput("SwerveDrive/currentModuleSates", currentModuleStates);

        for (int i = 0; i < 3; i++) {
            loggerInputs.currentSpeeds[i] =
                    Utils.chassisSpeedsToArray(
                            kinematics.toChassisSpeeds(
                                    currentModuleStates[0],
                                    currentModuleStates[1],
                                    currentModuleStates[2],
                                    currentModuleStates[3]
                            ))[i];
        }

        loggerInputs.linearVelocity = Math.hypot(loggerInputs.currentSpeeds[0], loggerInputs.currentSpeeds[1]);

        acceleration.update(loggerInputs.linearVelocity);
        loggerInputs.acceleration = accelFilter.calculate(acceleration.get());

        loggerInputs.supplyCurrent =
                modules[0].getSupplyCurrent() + modules[1].getSupplyCurrent() + modules[2].getSupplyCurrent() + modules[3].getStatorCurrent();

        loggerInputs.statorCurrent =
                modules[0].getStatorCurrent() + modules[1].getStatorCurrent() + modules[2].getStatorCurrent() + modules[3].getStatorCurrent();

        loggerInputs.rawYaw = gyro.getRawYaw();
        loggerInputs.yaw = gyro.getYaw();
        loggerInputs.pitch = gyro.getPitch();
        gyro.updateInputs(loggerInputs);

        SwerveDriveKinematics.desaturateWheelSpeeds(Utils.arrayToSwerveModuleStates(loggerInputs.desiredModuleStates), SwerveConstants.MAX_X_Y_VELOCITY); //TODO: may not work
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(Utils.arrayToSwerveModuleStates(loggerInputs.desiredModuleStates)[i]);
        }

        Logger.processInputs("SwerveDrive", loggerInputs);
    }
}
