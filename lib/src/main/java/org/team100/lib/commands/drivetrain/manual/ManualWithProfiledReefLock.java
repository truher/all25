package org.team100.lib.commands.drivetrain.manual;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The profile depends on robot speed, making rotation the lowest priority.
 */
public class ManualWithProfiledReefLock implements FieldRelativeDriver {
    // don't try to go full speed
    private static final double PROFILE_SPEED = 0.6;
    // accelerate gently to avoid upset
    private static final double PROFILE_ACCEL = 0.5;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveDriveSubsystem m_drive;

    private final Supplier<Boolean> m_lockToReef;

    private final Feedback100 m_thetaFeedback;

    // LOGGERS
    private final BooleanLogger m_log_snap_mode;
    private final DoubleLogger m_log_max_speed;
    private final DoubleLogger m_log_max_accel;
    private final DoubleLogger m_log_goal_theta;
    private final Control100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_output_omega;
    private final Supplier<Boolean> m_lockToFunnel;
    
    // package private for testing
    Rotation2d m_goal = null;
    Control100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     * @param thetaController
     * @param omegaController
     */
    public ManualWithProfiledReefLock(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Boolean> lockToReef,
            Feedback100 thetaController,
            SwerveDriveSubsystem drive,
            Supplier<Boolean> lockToFunnel) {
        LoggerFactory child = parent.child(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_lockToReef = lockToReef;
        m_drive = drive;
        m_thetaFeedback = thetaController;
        m_log_snap_mode = child.booleanLogger(Level.TRACE, "snap mode");
        m_log_max_speed = child.doubleLogger(Level.TRACE, "maxSpeedRad_S");
        m_log_max_accel = child.doubleLogger(Level.TRACE, "maxAccelRad_S2");
        m_log_goal_theta = child.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = child.control100Logger(Level.TRACE, "setpoint/theta");
        m_log_theta_FF = child.doubleLogger(Level.TRACE, "thetaFF");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "thetaFB");
        m_log_output_omega = child.doubleLogger(Level.TRACE, "output/omega");
        m_lockToFunnel  = lockToFunnel;
    }

    @Override
    public void reset(SwerveModel state) {
        m_thetaSetpoint = state.theta().control();
        m_goal = null;
        m_thetaFeedback.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * If you touch the POV and not the twist rotation, it remembers the POV. if you
     * use the twist rotation, it forgets and just uses that.
     * 
     * Desaturation prefers the rotational profile completely in the snap case, and
     * normally in the non-snap case.
     * 
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public FieldRelativeVelocity apply(
            final SwerveModel state,
            final DriverControl.Velocity twist1_1) {
        final FieldRelativeVelocity control = clipAndScale(twist1_1);
        // System.out.println("AHHH");
        final double currentVelocity = state.velocity().norm();

        final TrapezoidProfile100 m_profile = makeProfile(currentVelocity);

        if(!m_lockToReef.get() && !m_lockToFunnel.get()){
            m_goal = null;
        } else{
            if(m_lockToReef.get()) {
                Rotation2d rotationToReef = FieldConstants.angleToReefCenter(m_drive.getPose().getTranslation());
                // m_goal = FieldConstants.getSectorAngle(currentFieldSector).rotateBy(Rotation2d.k180deg);
                m_goal = rotationToReef;
                // System.out.println(currentFieldSector);
            }

            if(m_lockToFunnel.get()){
                Pose2d goalTranslationLeft = new Pose2d(1.2, 7.00, Rotation2d.fromDegrees(-54));
                Pose2d goalTranslationRight = new Pose2d(1.2, 1.05, Rotation2d.fromDegrees(54));
    
                ArrayList<Pose2d> poses = new ArrayList<>();
    
                poses.add(goalTranslationLeft);
                poses.add(goalTranslationRight);
    
                m_goal = m_drive.getPose().nearest(poses).getRotation();
            }
        }

         
        // m_goal = Rotation2d.fromDegrees(90);

        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_log_snap_mode.log(() -> false);
            return control;
        }

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            m_thetaSetpoint = state.theta().control();
        }

        final double thetaFB = m_thetaFeedback.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // feedforward uses the new setpoint
        //

        final double yawMeasurement = state.theta().x();
        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()));
        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        final Model100 goalState = new Model100(m_goal.getRadians(), 0);

        // use the modulus closest to the measurement
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(yawMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goalState);

        // the snap overrides the user input for omega.
        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
                
        FieldRelativeVelocity twistWithSnapM_S = new FieldRelativeVelocity(control.x(), control.y(), omega);

        m_log_snap_mode.log(() -> true);
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_output_omega.log(() -> omega);


        return twistWithSnapM_S;
    }

    public FieldRelativeVelocity clipAndScale(DriverControl.Velocity twist1_1) {
        // clip the input to the unit circle
        final DriverControl.Velocity clipped = DriveUtil.clampTwist(twist1_1, 1.0);

 

        // scale to max in both translation and rotation
        return DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    /**
     * Note that the max speed and accel are inversely proportional to the current
     * velocity.
     */
    public TrapezoidProfile100 makeProfile(double currentVelocity) {
        // fraction of the maximum speed
        final double xyRatio = Math.min(1, currentVelocity / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        final double oRatio = 1 - xyRatio;
        // add a little bit of default speed
        final double kRotationSpeed = Math.max(0.1, oRatio);

        final double maxSpeedRad_S = m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kRotationSpeed * PROFILE_SPEED;

        final double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * kRotationSpeed * PROFILE_ACCEL;

        m_log_max_speed.log(() -> maxSpeedRad_S);
        m_log_max_accel.log(() -> maxAccelRad_S2);

        return new TrapezoidProfile100(
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);
    }
}
