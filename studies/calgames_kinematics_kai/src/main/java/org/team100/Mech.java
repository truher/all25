package org.team100;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mech extends SubsystemBase {
    private final double m_armLength;
    private final double m_handLength;
    private final Kinematics m_kinematics;
    /**
     * The current mechanism joint configuration. For a real mechanism this would be
     * represented by the sensors of the real mechanism.
     */
    private Config m_config;

    public Mech(double armLength, double handLength) {
        m_armLength = armLength;
        m_handLength = handLength;
        m_kinematics = new Kinematics(armLength, handLength, 2.0, 0, 0, 0, 0);
        // in reality you wouldn't be able to just choose a default
        // configuration, you'd have to do something with the sensors.
        m_config = new Config(0.5, 0, 0);
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
        return m_config;
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

    private void set(Config config) {
        m_config = config;
    }

    private void addConfig(double height, double shoulder, double wrist) {
        set(new Config(
                m_config.shoulderHeight() + height,
                m_config.shoulderAngle() + shoulder,
                m_config.wristAngle() + wrist));
    }

    private void addCartesian(double x, double y, double r) {
        set(m_kinematics.inverse(
                m_kinematics.forward(m_config)
                        .plus(new Transform2d(x, y, new Rotation2d(r)))));
    }

}
