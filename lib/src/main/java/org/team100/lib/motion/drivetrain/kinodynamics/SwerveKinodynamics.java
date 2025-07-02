package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Kinematics and dynamics of the swerve drive.
 * 
 * Includes speed limits, dynamic constraints, and kinematics.
 * 
 * This class represents *absolute maxima.*
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamics implements Glassy {
    // geometry
    private final double m_fronttrack;
    private final double m_backtrack;
    private final double m_wheelbase;
    private final double m_frontoffset;
    private final double m_radius;
    private final double m_vcg;
    private final SwerveDriveKinematics100 m_kinematics;

    // configured inputs
    private final double m_maxDriveVelocityM_S;
    private final double m_stallAccelerationM_S2;
    private final double m_maxDriveAccelerationM_S2;
    private final double m_maxDriveDecelerationM_S2;
    private final double m_maxSteeringVelocityRad_S;

    // calculated
    private final double m_maxAngleSpeedRad_S;
    private final double m_maxAngleAccelRad_S2;
    private final double m_maxAngleStallAccelRad_S2;
    private final Profile100 m_steeringProfile;
    // fulcrum is the distance from the center to the nearest edge.
    private final double m_fulcrum;

    /**
     * @param maxDriveVelocity        module drive speed m/s
     * @param stallAcceleration       acceleration at stall, used to compute
     *                                back-EMF-limited acceleration at higher RPMs,
     *                                resulting in an exponential velocity curve at
     *                                max output.
     * @param maxDriveAcceleration    module drive accel m/s^2, used for
     *                                constant-acceleration profiles. This should be
     *                                less than the stall acceleration, so that the
     *                                robot can stay ahead of the profile initially,
     *                                and fall behind as speed increases.
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than accel limit, this is a positive number.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param fronttrack              meters
     * @param backtrack               meters
     * @param wheelbase               meters
     * @param frontoffset             distance from the center of mass to the front
     *                                wheels, meters
     * @param vcg                     vertical center of gravity, meters
     */
    SwerveKinodynamics(
            double maxDriveVelocity,
            double stallAcceleration,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double maxSteeringVelocity,
            double maxSteeringAcceleration,
            double fronttrack,
            double backtrack,
            double wheelbase,
            double frontoffset,
            double vcg) {
        if (fronttrack < 0.1 || backtrack < 0.1)
            throw new IllegalArgumentException();
        if (wheelbase < 0.1)
            throw new IllegalArgumentException();

        m_fronttrack = fronttrack;
        m_backtrack = backtrack;
        m_wheelbase = wheelbase;
        m_frontoffset = frontoffset;
        // fulcrum is the distance from the center to the nearest edge.
        m_fulcrum = Math.min(Math.min(m_fronttrack, m_backtrack) / 2, m_wheelbase / 2);
        m_vcg = vcg;
        // distance from center to wheel
        m_radius = Math.hypot((fronttrack + backtrack) / 4, m_wheelbase / 2);
        m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(m_frontoffset, m_fronttrack / 2),
                new Translation2d(m_frontoffset, -m_fronttrack / 2),
                new Translation2d(m_frontoffset - m_wheelbase, m_backtrack / 2),
                new Translation2d(m_frontoffset - m_wheelbase, -m_backtrack / 2));

        m_maxDriveVelocityM_S = maxDriveVelocity;

        m_stallAccelerationM_S2 = stallAcceleration;
        m_maxDriveAccelerationM_S2 = maxDriveAcceleration;
        m_maxDriveDecelerationM_S2 = maxDriveDeceleration;
        m_maxSteeringVelocityRad_S = maxSteeringVelocity;
        m_maxAngleSpeedRad_S = m_maxDriveVelocityM_S / m_radius;

        double accel = Math.max(m_maxDriveAccelerationM_S2, m_maxDriveDecelerationM_S2);
        m_maxAngleAccelRad_S2 = 12 * accel * m_radius
                / (m_fronttrack * m_backtrack + m_wheelbase * m_wheelbase);

        m_maxAngleStallAccelRad_S2 = 12 * m_stallAccelerationM_S2 * m_radius
                / (m_fronttrack * m_backtrack + m_wheelbase * m_wheelbase);

        m_steeringProfile = new TrapezoidProfile100(
                m_maxSteeringVelocityRad_S,
                maxSteeringAcceleration,
                0.02); // one degree
    }

    public Profile100 getSteeringProfile() {
        return m_steeringProfile;
    }

    /** Cruise speed, m/s. */
    public double getMaxDriveVelocityM_S() {
        return m_maxDriveVelocityM_S;
    }

    /**
     * Acceleration at stall, without current limiting. Used to compute
     * back-EMF-limited torque available at higher RPMs.
     */
    public double getStallAccelerationM_S2() {
        return m_stallAccelerationM_S2;
    }

    public double getMaxAngleStallAccelRad_S2() {
        return m_maxAngleStallAccelRad_S2;
    }

    /**
     * Motor-torque-limited acceleration rate, m/s^2. Used for constant-acceleration
     * profiles.
     */
    public double getMaxDriveAccelerationM_S2() {
        return m_maxDriveAccelerationM_S2;
    }

    /**
     * Motor-torque-limited drive deceleration rate, m/s^2. Motors are better at
     * slowing down than speeding up, so this should be larger than the accel rate.
     */
    public double getMaxDriveDecelerationM_S2() {
        return m_maxDriveDecelerationM_S2;
    }

    /** Cruise speed of the swerve steering axes, rad/s. */
    public double getMaxSteeringVelocityRad_S() {
        return m_maxSteeringVelocityRad_S;
    }

    /** Spin cruise speed, rad/s. Computed from drive and frame size. */
    public double getMaxAngleSpeedRad_S() {
        return m_maxAngleSpeedRad_S;
    }

    /**
     * Motor-torque-limited spin accel rate, rad/s^2. Computed from drive and frame
     * size.
     */
    public double getMaxAngleAccelRad_S2() {
        return m_maxAngleAccelRad_S2;
    }

    /**
     * Acceleration which will tip the robot onto two wheels, m/s^2. Computed from
     * vertical center of gravity and frame size.
     */
    public double getMaxCapsizeAccelM_S2() {
        return 9.8 * (m_fulcrum / m_vcg);
    }

    /**
     * Inverse kinematics, chassis speeds => module states.
     * 
     * The resulting state speeds are always positive.
     * 
     * This version does **DISCRETIZATION** to correct for swerve veering.
     * 
     * It also does extra veering correction proportional to rotation rate and
     * translational acceleration.
     * 
     * States may include empty angles for motionless wheels.
     */
    public SwerveModuleStates toSwerveModuleStates(ChassisSpeeds in) {
        return toSwerveModuleStates(in, TimedRobot100.LOOP_PERIOD_S);
    }

    /**
     * Discretizes.
     * 
     * States may include empty angles for motionless wheels.
     * Otherwise angle is always within [-pi, pi].
     */
    SwerveModuleStates toSwerveModuleStates(ChassisSpeeds in, double dt) {
        // This is the extra correction angle ...
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(in.omega));
        // ... which is subtracted here; this isn't really a field-relative
        // transformation it's just a rotation.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                in.vx, in.vy, in.omega).toRobotRelative(angle);
        // discretization does not affect omega
        DiscreteSpeed descretized = discretize(chassisSpeeds, dt);
        SwerveModuleStates states = m_kinematics.toSwerveModuleStates(descretized);
        return states;
    }

    /**
     * Given a desired instantaneous speed, extrapolate ahead one step, and return
     * the twist required to achieve that state.
     */
    public static DiscreteSpeed discretize(ChassisSpeeds chassisSpeeds, double dt) {
        Pose2d desiredDeltaPose = new Pose2d(
                chassisSpeeds.vx * dt,
                chassisSpeeds.vy * dt,
                new Rotation2d(chassisSpeeds.omega * dt));

        return new DiscreteSpeed(Pose2d.kZero.log(desiredDeltaPose), dt);
    }

    /**
     * Returns the "instantaneous" chassis speeds corresponding to the module
     * states, i.e. the chassis speed pointing at the result of applying the module
     * state twist.
     * 
     * This could be used with odometry, but because odometry uses module positions
     * instead of velocities, it is not needed.
     * 
     * It performs inverse discretization and an extra correction.
     */
    public ChassisSpeeds toChassisSpeedsWithDiscretization(
            SwerveModuleStates moduleStates,
            double dt) {
        ChassisSpeeds discreteSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
        Twist2d twist = new Twist2d(
                discreteSpeeds.vx * dt,
                discreteSpeeds.vy * dt,
                discreteSpeeds.omega * dt);

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        double omega = discreteSpeeds.omega;
        // This is the opposite direction
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(omega));
        return new ChassisSpeeds(
                continuousSpeeds.vx,
                continuousSpeeds.vy,
                continuousSpeeds.omega)
                .toRobotRelative(angle.unaryMinus());
    }

    public SwerveDrivePoseEstimator100 newPoseEstimator(
            LoggerFactory parent,
            Gyro gyro,
            SwerveModulePositions modulePositions,
            Pose2d initialpose,
            double timestampSeconds) {
        return new SwerveDrivePoseEstimator100(
                parent,
                this,
                gyro,
                modulePositions,
                initialpose,
                timestampSeconds);
    }

    /**
     * Robot-relative speed, without discretization.
     * This simply rotates the velocity from the field frame to the robot frame.
     */
    public static ChassisSpeeds toInstantaneousChassisSpeeds(
            FieldRelativeVelocity v,
            Rotation2d theta) {
        return new ChassisSpeeds(
                v.x(),
                v.y(),
                v.theta())
                .toRobotRelative(theta);

    }

    /**
     * Field-relative speed, without discretization.
     * This simply rotates the velocity from the robot frame to the field frame.
     */
    public static FieldRelativeVelocity fromInstantaneousChassisSpeeds(ChassisSpeeds instantaneous, Rotation2d theta) {
        ChassisSpeeds c = instantaneous.toFieldRelative(theta);
        return new FieldRelativeVelocity(c.vx, c.vy, c.omega);
    }

    public SwerveDriveKinematics100 getKinematics() {
        return m_kinematics;
    }

}