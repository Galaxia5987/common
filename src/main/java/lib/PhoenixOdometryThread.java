package lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.swerve.SwerveConstants;
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
  private StatusSignal<Double>[] signals = new StatusSignal[0];
  private StatusSignal<Double>[] signalSlopes = new StatusSignal[0];
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
      StatusSignal<Double>[] newSignals = new StatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;

      StatusSignal<Double>[] newSignalSlopes = new StatusSignal[signalSlopes.length + 1];
      System.arraycopy(signalSlopes, 0, newSignalSlopes, 0, signalSlopes.length);
      newSignals[signalSlopes.length] = signalSlope;
      signalSlopes = newSignalSlopes;

      queues.add(queue);
    } finally {
      signalsLock.unlock();
      SwerveDrive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD) {
          BaseStatusSignal.waitForAll(2.0 / SwerveConstants.ODOMETRY_FREQUENCY, signals);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing. No reasoning for this behavior
          // is provided by the documentation.
          sleep((long) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      SwerveDrive.odometryLock.lock();
      try {
        for (int i = 0; i < signals.length; i++) {
          double value = BaseStatusSignal.getLatencyCompensatedValue(
                  signals[i], signalSlopes[i]
          );
          queues.get(i).offer();
        }
      } finally {
        SwerveDrive.odometryLock.unlock();
      }
    }
  }
}