package org.team100.lib.visualization;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Simulated projectile in XY plane, uses constant velocity, continues forever.
 * 
 * Provides Field2d visualization using the name "ball".
 */
public class BallR2 {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final DoubleArrayLogger m_log_field_ball;
    private final Supplier<ModelR3> m_robot;
    private final Supplier<Rotation2d> m_azimuth;
    /** Projectile speed m/s */
    private final double m_speed;

    // null when contained in robot.
    Translation2d m_location;
    GlobalVelocityR2 m_velocity;

    /**
     * @param field   log
     * @param robot   state (pose2d, velocityR3)
     * @param azimuth absolute
     * @param speed   muzzle speed
     */
    public BallR2(
            LoggerFactory field,
            Supplier<ModelR3> robot,
            Supplier<Rotation2d> azimuth,
            double speed) {
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_robot = robot;
        m_azimuth = azimuth;
        m_speed = speed;
    }

    /** Sets initial position and velocity. */
    void launch() {
        // Velocity due only to the gun
        GlobalVelocityR2 v = GlobalVelocityR2.fromPolar(m_azimuth.get(), m_speed);
        // Velocity due to robot translation
        GlobalVelocityR2 mv = GlobalVelocityR2.fromSe2(m_robot.get().velocity());
        // Initial position is at the robot center.  TODO: offsets.
        m_location = m_robot.get().pose().getTranslation();
        // Initial velocity.
        m_velocity = v.plus(mv);
    }

    /** Evolves state one time step. */
    void fly() {
        m_location = m_velocity.integrate(m_location, DT);
    }

    private void reset() {
        m_location = null;
    }

    /** Shoot the ball and continue its path as long as the command runs. */
    public Command shoot() {
        return Commands.startRun(this::launch, this::fly)
                .finallyDo(this::reset);
    }

    public void periodic() {
        m_log_field_ball.log(this::poseArray);
    }

    private double[] poseArray() {
        Translation2d t = location();
        return new double[] { t.getX(), t.getY(), 0 };
    }

    private Translation2d location() {
        if (m_location == null) {
            // The ball is riding with the robot.
            return m_robot.get().translation();
        }
        return m_location;
    }
}
