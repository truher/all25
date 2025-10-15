package org.team100.ballerina;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Provide a pose estimate with manual control. */
public class ManualPose {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private static final double MAX_V = 1.0;
    private static final double MAX_OMEGA = 1.0;
    private final DoubleArrayLogger m_log_field_robot;
    private final Supplier<Velocity> m_v;
    private final CotemporalCache<Pose2d> m_poseCache;
    /** Used only by update(). */
    private Pose2d m_pose;

    public ManualPose(
            LoggerFactory fieldLogger,
            Supplier<Velocity> v,
            Pose2d initial) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_v = v;
        m_pose = initial;
        m_poseCache = Cache.of(this::update);
    }

    public Pose2d getPose() {
        return m_poseCache.get();
    }

    public SwerveModel getState() {
        return new SwerveModel(getPose());
    }

    public void periodic() {
        m_log_field_robot.log(this::poseArray);
    }

    private double[] poseArray() {
        Pose2d pose = m_poseCache.get();
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees() };
    }

    private Pose2d update() {
        Velocity v = m_v.get();
        m_pose = new Pose2d(
                m_pose.getX() + v.x() * MAX_V * DT,
                m_pose.getY() + v.y() * MAX_V * DT,
                m_pose.getRotation().plus(new Rotation2d(v.theta() * MAX_OMEGA * DT)));
        return m_pose;
    }
}
