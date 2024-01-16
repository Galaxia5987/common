// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
package lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.swerve.SwerveConstantsTalonFX;
import frc.robot.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock =
            new ReentrantLock(); // Prevents conflicts when registering signals
    private final List<StatusSignal<Double>> signals = new ArrayList<>();
    private final List<StatusSignal<Double>> signalSlopes = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private boolean isCANFD = false;

    private static PhoenixOdometryThread instance = null;

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal, StatusSignal<Double> signalSlope) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        signalsLock.lock();
        SwerveDrive.odometryLock.lock();
        try {
            isCANFD = CANBus.isNetworkFD(device.getNetwork());

            signals.add(signal);
            signalSlopes.add(signalSlope);

            queues.add(queue);
        } finally {
            signalsLock.unlock();
            SwerveDrive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        List<StatusSignal<Double>> all = new ArrayList<>() {{
            addAll(signals);
            addAll(signalSlopes);
        }};
        BaseStatusSignal[] allArr = all.toArray(new BaseStatusSignal[0]);

        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD) {
                    BaseStatusSignal.waitForAll(2.0 / SwerveConstantsTalonFX.ODOMETRY_FREQUENCY,
                            allArr);
                } else {
                    // "waitForAll" does not support blocking on multiple
                    // signals with a bus that is not CAN FD, regardless
                    // of Pro licensing. No reasoning for this behavior
                    // is provided by the documentation.
                    sleep((long) (1000.0 / SwerveConstantsTalonFX.ODOMETRY_FREQUENCY));
                    if (!signals.isEmpty()) {
                        signals.forEach(StatusSignal::refresh);
                        signalSlopes.forEach(StatusSignal::refresh);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            SwerveDrive.odometryLock.lock();
            try {
                for (int i = 0; i < signals.size(); i++) {
                    double value = BaseStatusSignal.getLatencyCompensatedValue(
                            signals.get(i), signalSlopes.get(i)
                    );
                    queues.get(i).offer(value);
                }
            } finally {
                SwerveDrive.odometryLock.unlock();
            }
        }
    }
}