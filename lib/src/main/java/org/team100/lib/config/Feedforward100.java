package org.team100.lib.config;

/**
 * The motor feedforward model includes four constants.
 * 
 * @param kV  Velocity: ratio between unloaded speed (rev/s) and voltage, so the
 *            units are VOLT-SEC/REV. This reflects the "Back EMF" aspect of a
 *            motor: it produces a voltage proportional to speed. The value is
 *            an intrinsic property of the motor.
 * @param kA  Acceleration: ratio between acceleration (rev/s^2) and voltage, so
 *            the units are VOLT-SEC^2/REV. This reflects torque production in
 *            the motor: torque is proportional to current, which is
 *            proportional to (net) voltage. The value will depend on the
 *            inertia of the mechanism.
 * @param kD  Deceleration: like kA but when the motor is braking, i.e.
 *            acceleration is opposite to the current speed. Motors seem to
 *            decelerate much better than they accelerate -- perhaps the current
 *            limit acts differently (or not at all)?
 * @param kSS Static friction: voltage to get the motor moving from a stop, so
 *            the units are VOLTS. The value will depend on the "stickiness" of
 *            the mechanism.
 * @param kDS Dynamic friction: voltage to just barely keep the motor moving,
 *            the units are VOLTS. The value will depend on the "viscosity" of
 *            the mechanism.
 * 
 * @see https://en.wikipedia.org/wiki/Motor_constants
 * @see {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} which uses
 *      a similar model, and also a discrete one which is more accurate.
 * @see {@link org.team100.lib.config.FeedforwardTest} which compares the
 *      models.
 */
public class Feedforward100 {
    private final double kV;
    private final double kA;
    // TODO: make kD actually work
    @SuppressWarnings("unused")
    private final double kD;
    private final double kSS;
    private final double kDS;
    private final double staticFrictionSpeedLimit;

    Feedforward100(
            double kV,
            double kA,
            double kD,
            double kSS,
            double kDS,
            double staticFrictionSpeedLimit) {
        this.kV = kV;
        this.kA = kA;
        this.kD = kD;
        this.kSS = kSS;
        this.kDS = kDS;
        this.staticFrictionSpeedLimit = staticFrictionSpeedLimit;
    }

    public static Feedforward100 makeNeo() {
        return new Feedforward100(0.122, 0.000, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeNeoArm() {
        return new Feedforward100(0.22, 1, 1, 0, 0, 0);
    }

    public static Feedforward100 makeNeo550() {
        return new Feedforward100(0.12, 0, 0, 0, .07, 0);
    }

    public static Feedforward100 makeArmPivot() {
        return new Feedforward100(
                0.09,
                0.005,
                0.005,
                0.100,
                0.065,
                0.5);
    }

    public static Feedforward100 makeClimber() {
        return new Feedforward100(
                0.14,
                0.05,
                0.05,
                0.100,
                0.1,
                0.0);
    }

    public static Feedforward100 zero() {
        return new Feedforward100(0, 0, 0, 0, 0, 0);
    }

    public static Feedforward100 makeNeoVortex() {
        return new Feedforward100(0.122, 0.000, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeWCPSwerveTurningFalcon() {
        return new Feedforward100(0.110, 0.000, 0.000, 0.180, 0.010, 0.5);
    }

    /**
     * 9/24/04
     * Voltage feedforward for steering motors in air.
     * Tuned in air, not on carpet, so probably the velocity number is too low.
     */
    public static Feedforward100 makeWCPSwerveTurningFalcon6() {
        return new Feedforward100(0.150, 0.010, 0.010, 0.080, 0.100, 0.5);
    }

    public static Feedforward100 makeKrakenTurret() {
        return new Feedforward100(0.150, 0.010, 0.010, 0.080, 0.100, 0.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon() {
        return new Feedforward100(0.110, 0.000, 0.000, 0.375, 0.270, 0.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon6() {
        return new Feedforward100(0.13, 0.017, 0.017, 0, 0.26, 0.06);
    }

    public static Feedforward100 makeWCPSwerveDriveKraken6() {
        return new Feedforward100(0.13, 0.022, 0.007, 0.26, 0.26, 0.06); //WAS 0.115
    }

    public static Feedforward100 makeAMSwerveDriveFalcon6() {
        return new Feedforward100(0.110, 0.000, 0.000, 0.180, 0.010, 0.1);
    }

    public static Feedforward100 makeSimple() {
        return new Feedforward100(0.100, 0.100, 0.100, 0.100, 0.100, 0.1);
    }

    public static Feedforward100 makeShooterFalcon6() {
        return new Feedforward100(0.110, 0.000, 0.000, 0.000, 0.900, 0.1);
    }

    public static Feedforward100 makeKraken6Elevator() {
        return new Feedforward100(0.135, 0.005, 0.005, 0.005, 0.005, 0.1);
        // return new Feedforward100(0, 0, 0, 0, 0);

    }

    public static Feedforward100 makeKraken6Wrist() {
        return new Feedforward100(0.137, 0.0085, 0.002, 0.11, 0.11, 0.1);
        // return new Feedforward100(0.055, 0.025, 0.015, 0.005, 0.005, 0.1);

    }

    public static Feedforward100 makeKraken6WristWithLowerKd() {
        return new Feedforward100(0.14, 0.013, 0.005, 0.2, 0.23, 0.1);
        // return new Feedforward100(0, 0, 0, 0, 0);
    }

    public double velocityFFVolts(double motorRev_S) {
        return kV * motorRev_S;
    }

    /**
     * Uses kA all the time, because the resulting feedforward had a strange "tooth"
     * in it, maybe because of the use of the measured velocity rather than the
     * setpoint velocity?
     * 
     * TODO: make kD actually work?
     * Uses kA when speed and accel are in the same direction.
     * Uses kD when speed and accel are opposite.
     */
    public double accelFFVolts(double currentMotorRev_S, double motorRev_S_S) {
        // if ((currentMotorRev_S >= 0 && motorRev_S_S >= 0) || (currentMotorRev_S <= 0
        // && motorRev_S_S <= 0))
        // return kA * motorRev_S_S;
        // return kD * motorRev_S_S;
        return kA * motorRev_S_S;
    }

    public double frictionFFVolts(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (Math.abs(currentMotorRev_S) < staticFrictionSpeedLimit) {
            return kSS * direction;
        }
        return kDS * direction;
    }

    public static Feedforward100 makeTest1() {
        return new Feedforward100(0.3, 0.000, 0.000, 0.000, 0.100, 3.5);
    }
}
