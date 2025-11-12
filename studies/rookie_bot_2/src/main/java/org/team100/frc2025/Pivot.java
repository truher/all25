package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.Gravity;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Similar to RotaryPositionSubsystem1d but uses outboard positional control,
 * which is what we always do if we can.
 */
public class Pivot extends SubsystemBase {
    /** Home position, rad */
    private static final double HOME = 0.1;
    /** Extended position, rad */
    private static final double EXTEND = 1.4;

    private final Gravity m_gravity;
    private final AngularPositionServo m_servo;

    public Pivot(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_gravity = new Gravity(log,
                2.5, // Max gravity torque, Nm
                3 * Math.PI / 4); // Gravity torque position offset, rad
        IncrementalProfile profile = new TrapezoidIncrementalProfile(
                log,
                8, // max velocity rad/s
                13, // max accel rad/s^2 origin 1
                0.01); // tolerance
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(
                log,
                () -> profile,
                0.01, // position tolerance, rad
                0.01); // velocity tolerance, rad/s
        Feedforward100 ff = Feedforward100.makeNeo2(log);
        PIDConstants pid = PIDConstants.makePositionPID(log, 0.5);
        CanId canId = new CanId(5);
        BareMotor motor = NeoCANSparkMotor.get(log, canId, MotorPhase.FORWARD, 40, ff, pid);
        double initialPositionRad = 0;
        double gearRatio = 15;
        double minPositionRad = 0;
        double maxPositionRad = 1.5;
        m_servo = OutboardAngularPositionServo.make(
                log, motor, ref, initialPositionRad, gearRatio, minPositionRad, maxPositionRad);
        m_servo.reset();
    }

    public double getWrappedPositionRad() {
        return m_servo.getWrappedPositionRad();
    }

    /**
     * Go home (with gravity) and then hold position against the hard stop (without
     * gravity) forever.
     */
    public Command home() {
        Command goHome = run(() -> m_servo.setPositionProfiled(HOME, gravityTorque()));
        Command stayHome = run(() -> m_servo.setPositionProfiled(HOME, 0));
        return goHome.until(m_servo::atGoal).andThen(stayHome);
    }

    /** Go to the extended position, and hold there forever. */
    public Command extend() {
        return run(() -> m_servo.setPositionProfiled(EXTEND, gravityTorque()));
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    /////////////////////////////////////////////////////////////////

    private double gravityTorque() {
        return -1 * m_gravity.applyAsDouble(m_servo.getWrappedPositionRad());
    }
}
