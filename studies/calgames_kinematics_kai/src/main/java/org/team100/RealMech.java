package org.team100;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A version of the arm mechanism with real motors. */
public class RealMech extends SubsystemBase {
    private final double m_armLength;
    private final double m_handLength;
    private final JoelsKinematics m_kinematics;
    private final Jacobian m_jacobian;
    /**
     * The current mechanism joint configuration. For a real mechanism this would be
     * represented by the sensors of the real mechanism.
     */
    private final LinearMechanism m_elevator;
    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;
    private final Gravity m_gravity;

    public RealMech(LoggerFactory parent, double armLength, double handLength) {
        m_armLength = armLength;
        m_handLength = handLength;
        m_kinematics = new JoelsKinematics(armLength, handLength);
        m_jacobian = new Jacobian(m_kinematics);

        m_gravity = Gravity.from2025();
        LoggerFactory elevatorLog = parent.name("elevator");
        LoggerFactory shoulderLog = parent.name("shoulder");
        LoggerFactory wristLog = parent.name("wrist");
        switch (Identity.instance) {
            case COMP_BOT -> {
                Kraken6Motor elevatorMotor = new Kraken6Motor();
                Talon6Encoder elevatorEncoder = new Talon6Encoder(
                        elevatorLog, elevatorMotor);
                m_elevator = new LinearMechanism(
                        elevatorLog,
                        elevatorMotor,
                        elevatorEncoder, handLength,
                        handLength, armLength, handLength);
                m_shoulder = new RotaryMechanism(
                        parent.name("shoulder"), null, null, handLength,
                        armLength, handLength);
                m_wrist = new RotaryMechanism(
                        parent.name("wrist"), null, null,
                        handLength, armLength, handLength);
            }
            default -> {
                SimulatedBareMotor elevatorMotor = new SimulatedBareMotor();
                SimulatedBareEncoder elevatorEncoder = new SimulatedBareEncoder();
                m_elevator = new LinearMechanism(
                        elevatorLog, null, null, handLength,
                        handLength, armLength, handLength);
                m_shoulder = new RotaryMechanism(
                        shoulderLog, null, null, handLength,
                        armLength, handLength);
                m_wrist = new RotaryMechanism(
                        wristLog, null, null,
                        handLength, armLength, handLength);
            }
        }

    }

    public static Mech make2025() {
        return new Mech(0.3, 0.1);
    }

    public double getArmLength() {
        return m_armLength;
    }

    public double getHandLength() {
        return m_handLength;
    }

    public Config getConfig() {
        // TODO: remove these defaults
        return new Config(
                m_elevator.getPositionM().orElse(0),
                m_shoulder.getPositionRad().orElse(0),
                m_wrist.getPositionRad().orElse(0));
    }

    /** There are no profiles here, so this control needs to be feasible. */
    public void set(SwerveControl control) {
        // control has three inputs:
        // position
        Pose2d p = control.pose();
        Config c = m_kinematics.inverse(p);
        if (c.isNaN()) {
            System.out.println("skipping invalid config");
            stop();
            return;
        }
        // velocity
        FieldRelativeVelocity v = control.velocity();
        JointVelocities jv = m_jacobian.inverse(control.model());
        // force
        JointForce jf = m_gravity.get(c);
        // set each mechanism
        m_elevator.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), jv.shoulder(), 0, jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), jv.wrist(), 0, jf.wrist());
    }

    /** Controls are configuration axes. */
    public Command config(
            DoubleSupplier height,
            DoubleSupplier shoulder,
            DoubleSupplier wrist) {
        return run(() -> addConfig(
                height.getAsDouble(), shoulder.getAsDouble(), wrist.getAsDouble()));
    }

    /** Controls are cartesian axes. */
    public Command cartesian(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier r) {
        return run(() -> addCartesian(x.getAsDouble(), y.getAsDouble(), r.getAsDouble()));
    }

    /////////////////////////////////

    /** This is not "hold position" this is "torque off" */
    private void stop() {
        m_elevator.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    private void set(Config c) {
        // force
        JointForce jf = m_gravity.get(c);
        // set each mechanism, with zero velocity
        m_elevator.setPosition(c.shoulderHeight(), 0, 0, jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), 0, 0, jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), 0, 0, jf.wrist());
    }

    private void addConfig(double height, double shoulder, double wrist) {
        Config c = getConfig();
        set(new Config(
                c.shoulderHeight() + height,
                c.shoulderAngle() + shoulder,
                c.wristAngle() + wrist));
    }

    private void addCartesian(double x, double y, double r) {
        Transform2d t = new Transform2d(x, y, new Rotation2d(r));
        Pose2d p = m_kinematics.forward(getConfig());
        // Pose2d.add() method uses the pose frame so we don't use it.
        // We transform p in the global frame by adding components
        double x2 = p.getX() + t.getX();
        double y2 = p.getY() + t.getY();
        Rotation2d r2 = p.getRotation().plus(t.getRotation());
        Pose2d newP = new Pose2d(x2, y2, r2);
        Config c = m_kinematics.inverse(newP);
        if (c.isNaN()) {
            System.out.println("skipping invalid config");
            stop();
            return;
        }
        set(c);
    }

}
