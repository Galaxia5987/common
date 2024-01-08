package frc.robot.swerve;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Utils;
import lib.math.differential.Derivative;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive INSTANCE = null;
    private final GyroIO gyro;
    private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, RL, RR
    private final LinearFilter accelFilter = LinearFilter.movingAverage(15);
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    SwerveConstants.WHEEL_POSITIONS[0],
                    SwerveConstants.WHEEL_POSITIONS[1],
                    SwerveConstants.WHEEL_POSITIONS[2],
                    SwerveConstants.WHEEL_POSITIONS[3]);
    private final SwerveDriveOdometry odometry;
    private final Derivative acceleration = new Derivative();
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
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
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getYaw()), modulePositions);
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
    @AutoLogOutput
    public double getRawYaw() {
        return loggerInputs.rawYaw;
    }

    /**
     * Gets the yaw reading from the gyro with the calculated offset.
     *
     * @return Yaw angle with offset. [rad]
     */
    @AutoLogOutput
    public double getYaw() {
        return loggerInputs.yaw;
    }

    @AutoLogOutput
    public double getPitch() {
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

    @AutoLogOutput
    public Pose2d getBotPose() {
        return Utils.arrayToPose2d(loggerInputs.botPose);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @AutoLogOutput
    public double getVelocity() {
        return loggerInputs.linearVelocity;
    }

    @AutoLogOutput
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

    public Command checkModule(int moduleIndex){
        return new RunCommand(
                ()-> modules[moduleIndex].checkModule(), this);
    }

    public void checkSwerve() {
        for (int i = 0; i < 4; i++) {
            checkModule(i);
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

    @AutoLogOutput
    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public void updateOffsets(double[] offsets) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateOffset(offsets[i]);
        }
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
        loggerInputs.desiredSpeeds = Utils.chassisSpeedsToArray(chassisSpeeds);

        ChassisSpeeds fieldOrientedChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond,
                        new Rotation2d(getYaw()));

        if (new ChassisSpeeds(0, 0, 0).equals(chassisSpeeds)) {
            for (SwerveModule module : modules) {
                module.stopMotor();
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
     * @param xOutput percentage of the x speed
     * @param yOutput percentage of the y speed
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
        updateModulePositions();
        odometry.update(new Rotation2d(getYaw()), modulePositions);

        loggerInputs.botPose[0] = odometry.getPoseMeters().getX();
        loggerInputs.botPose[1] = odometry.getPoseMeters().getY();
        loggerInputs.botPose[2] = odometry.getPoseMeters().getRotation().getRadians();

        for (int i = 0; i < modules.length; i++) {
            loggerInputs.absolutePositions[i] = modules[i].getPosition();
            loggerInputs.currentModuleStates[i] = modules[i].getModuleState();
        }

        for (int i = 0; i < 3; i++) {
            loggerInputs.currentSpeeds[i] =
                    Utils.chassisSpeedsToArray(
                            kinematics.toChassisSpeeds(
                                    loggerInputs.currentModuleStates[0],
                                    loggerInputs.currentModuleStates[1],
                                    loggerInputs.currentModuleStates[2],
                                    loggerInputs.currentModuleStates[3]))[i];
        }

        loggerInputs.linearVelocity =
                Math.hypot(loggerInputs.currentSpeeds[0], loggerInputs.currentSpeeds[1]);

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
