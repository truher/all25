package org.team100.ballerina;

import java.util.function.Supplier;

import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Attempts to maintain aim. */
public class Turret extends SubsystemBase {
    private static final int GEAR_RATIO = 100;
    private static final double MIN_POSITION = -3;
    private static final double MAX_POSITION = 3;

    private final DoubleArrayLogger m_log_field_turret;

    private final Supplier<Pose2d> m_pose;
    private final Supplier<Translation2d> m_target;

    private final AngularPositionServo m_pivot;

    /**
     * @param parent Log
     * @param pose   Current robot pose, from the pose estimator.
     * @param target From the target designator; can change at any time.
     */
    public Turret(LoggerFactory parent, LoggerFactory field, Supplier<Pose2d> pose, Supplier<Translation2d> target) {
        LoggerFactory log = parent.type(this);
        m_log_field_turret = field.doubleArrayLogger(Level.COMP, "turret");

        m_pose = pose;
        m_target = target;

        IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 2, 0.05);
        ProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
        m_pivot = new OnboardAngularPositionServo(
                log, mech, ref, feedback);
        m_pivot.reset();

    }

    public boolean onTarget() {
        return false;

    }

    private void moveToAim() {
        Pose2d pose = m_pose.get();
        Translation2d target = m_target.get();
        Rotation2d absoluteBearing = target.minus(pose.getTranslation()).getAngle();
        Rotation2d relativeBearing = absoluteBearing.minus(pose.getRotation());
        System.out.printf("move to aim %6.3f\n", relativeBearing.getRadians());

        m_pivot.setPositionProfiled(relativeBearing.getRadians(), 0);
    }

    ////////////////////////////////////////////////////////
    ///
    /// COMMANDS

    public Command aim() {
        return run(() -> moveToAim());
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_field_turret.log(() -> new double[] {
                m_pose.get().getX(),
                m_pose.get().getY(),
                Math.toDegrees(m_pivot.getWrappedPositionRad()) });
    }

}
