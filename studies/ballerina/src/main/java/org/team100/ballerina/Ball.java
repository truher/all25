package org.team100.ballerina;

import java.util.function.Supplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Simulated projectile. */
public class Ball {
    private final DoubleArrayLogger m_log_field_ball;
    private final Supplier<SwerveModel> m_robot;
    private final Supplier<Rotation2d> m_azimuth;

    // null when contained in robot.
    private SwerveModel m_ball;

    public Ball(
            LoggerFactory field,
            Supplier<SwerveModel> robot,
            Supplier<Rotation2d> azimuth) {
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_robot = robot;
        m_azimuth = azimuth;
    }

    private void launch() {

    }

    private void fly() {

    }

    private void reset() {
        m_ball = null;
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
        SwerveModel m = m_ball == null ? m_robot.get() : m_ball;
        Translation2d t = m.translation();
        return new double[] { t.getX(), t.getY(), 0 };
    }
}
