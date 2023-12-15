package lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

/*
This class is used for conversions between angles with different
coordinate systems.

Each coordinate system is described as such:

    The vector representing the zero angle is called the ZeroVector,
    which can be anywhere between 0 and 2 * pi.

    The direction of positive increase in the angle is called ThetaDirection,
    which can be either clockwise or counter-clockwise.

The absolute/default coordinate system is `RIGHT_COUNTER_CLOCKWISE`,
all conversions to an absolute angle will be to this system.
 */
public class AngleUtil {

    public static final CoordinateSystem RIGHT_COUNTER_CLOCKWISE = CoordinateSystem.of(0, false);
    public static final CoordinateSystem RIGHT_CLOCKWISE = CoordinateSystem.of(0, true);
    public static final CoordinateSystem UP_COUNTER_CLOCKWISE = CoordinateSystem.of(Math.PI / 2, false);
    public static final CoordinateSystem UP_CLOCKWISE = CoordinateSystem.of(Math.PI / 2, true);
    public static final CoordinateSystem LEFT_COUNTER_CLOCKWISE = CoordinateSystem.of(Math.PI, false);
    public static final CoordinateSystem LEFT_CLOCKWISE = CoordinateSystem.of(Math.PI, true);
    public static final CoordinateSystem DOWN_COUNTER_CLOCKWISE = CoordinateSystem.of(3 * Math.PI / 2, false);
    public static final CoordinateSystem DOWN_CLOCKWISE = CoordinateSystem.of(3 * Math.PI / 2, true);

    public static double normalize(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle % (2 * Math.PI);
    }

    public static double absoluteAngleToYaw(double angle) {
        angle = normalize(angle);
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static Rotation2d absoluteAngleToYaw(Rotation2d angle) {
        return Rotation2d.fromRadians(absoluteAngleToYaw(angle.getRadians()));
    }

    public static double getAbsoluteAngle(ZeroVector zeroVector, ThetaDirection thetaDirection, double angle) {
        return getAbsoluteAngle(new CoordinateSystem(zeroVector, thetaDirection), angle);
    }

    public static double getAbsoluteAngle(double zeroVector, ThetaDirection thetaDirection, double angle) {
        return getAbsoluteAngle(new CoordinateSystem(zeroVector, thetaDirection), angle);
    }

    public static double getAbsoluteAngle(CoordinateSystem coordinateSystem, double angle) {
        return new Angle(coordinateSystem, angle).getAbsoluteAngle();
    }

    public enum ThetaDirection {
        COUNTER_CLOCKWISE(false),
        CLOCKWISE(true);

        public final boolean clockwise;

        ThetaDirection(boolean clockwise) {
            this.clockwise = clockwise;
        }

        public static ThetaDirection of(boolean invert) {
            return invert ? CLOCKWISE : COUNTER_CLOCKWISE;
        }

        public int get() {
            return clockwise ? -1 : 1;
        }

    }

    public static class Angle {
        public CoordinateSystem coordinateSystem;
        public double value;

        public Angle(CoordinateSystem coordinateSystem, double value) {
            this.coordinateSystem = coordinateSystem;
            this.value = normalize(value);
        }

        public Angle(CoordinateSystem coordinateSystem, Rotation2d value) {
            this(coordinateSystem, value.getRadians());
        }

        public Angle(double zeroVector, boolean clockwise, double value) {
            this(CoordinateSystem.of(zeroVector, clockwise), value);
        }

        public Angle(double zeroVector, boolean clockwise, Rotation2d value) {
            this(CoordinateSystem.of(zeroVector, clockwise), value);
        }

        public double getAbsoluteAngle() {
            double absoluteAngle =
                    coordinateSystem.zeroVector.zeroAbsoluteAngle +
                            coordinateSystem.thetaDirection.get() * value;
            return normalize(absoluteAngle);
        }

        public double minus(Angle other) {
            return getAbsoluteAngle() - other.getAbsoluteAngle();
        }

        public double plus(Angle other) {
            return getAbsoluteAngle() + other.getAbsoluteAngle();
        }

        @Override
        public String toString() {
            return "Angle: \n" +
                    "   Coordinate System: " +
                    coordinateSystem.zeroVector.toString() + ", " +
                    coordinateSystem.thetaDirection.name() + "\n" +
                    "   Value: " + value;
        }
    }

    public static class CoordinateSystem {
        public ZeroVector zeroVector;
        public ThetaDirection thetaDirection;

        public CoordinateSystem(ZeroVector zeroVector, ThetaDirection thetaDirection) {
            this.zeroVector = zeroVector;
            this.thetaDirection = thetaDirection;
        }

        public CoordinateSystem(double zeroVector, ThetaDirection thetaDirection) {
            this(new ZeroVector(zeroVector), thetaDirection);
        }

        public static CoordinateSystem of(double zeroVector, boolean clockwise) {
            return new CoordinateSystem(
                    new ZeroVector(zeroVector),
                    ThetaDirection.of(clockwise)
            );
        }
    }

    public static class ZeroVector {
        public static final double RIGHT = 0;
        public static final double UP = Math.PI / 2;
        public static final double LEFT = Math.PI;
        public static final double DOWN = 3 * Math.PI / 2;

        public final double zeroAbsoluteAngle;

        public ZeroVector(double zeroAbsoluteAngle) {
            this.zeroAbsoluteAngle = normalize(zeroAbsoluteAngle);
        }
    }
}
