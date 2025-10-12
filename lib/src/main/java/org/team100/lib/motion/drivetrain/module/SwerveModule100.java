package org.team100.lib.motion.drivetrain.module;

import java.util.Optional;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.state.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.state.SwerveModuleState100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Control of a single module.
 * 
 * Everything here is package-private because SwerveModuleCollection is the only
 * client.
 * 
 * Implements drive/steer coupling. The path from the drive motor to the wheel
 * involves a shaft on the steering axis; driving the wheel requires *relative*
 * motion of this shaft and the steering axis itself. So when the steering axis
 * moves, it affects the relative position of the drive motor and the drive
 * wheel.
 * 
 * TODO: this coupling implementation is, I think, totally broken.
 * 
 * There is some discussion of this topic here:
 * https://www.chiefdelphi.com/t/kcoupleratio-in-ctre-swerve/483380
 */
public abstract class SwerveModule100 {
    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;
    /**
     * The previous desired angle, used if the current desired angle is empty (i.e.
     * the module is motionless) and to calculate steering velocity.
     */
    private Rotation2d m_previousDesiredAngle;
    private double m_previousTime;

    protected SwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        // The initial previous angle is the measurement.
        m_previousDesiredAngle = new Rotation2d(m_turningServo.getWrappedPositionRad());
        m_previousTime = Takt.get();
    }

    /**
     * Optimizes.
     * 
     * Given an empty angle, it uses the previous one.
     */
    void setDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        desired = optimize(desired);
        actuate(desired);
    }

    /**
     * Does not optimize.
     * 
     * Given an empty angle, it uses the previous one.
     */
    void setRawDesiredState(SwerveModuleState100 desired) {
        desired = usePreviousAngleIfEmpty(desired);
        actuate(desired);
    }

    /** Make sure the setpoint and measurement are the same. */
    void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    void close() {
        m_turningServo.close();
    }

    /** FOR TEST ONLY */
    SwerveModuleState100 getState() {
        double driveVelocity = m_driveServo.getVelocity();
        double turningPosition = m_turningServo.getWrappedPositionRad();

        return new SwerveModuleState100(
                driveVelocity,
                Optional.of(new Rotation2d(turningPosition)));
    }

    /** Uses Cache so the position is fresh and coherent. */
    SwerveModulePosition100 getPosition() {
        double drive_M = m_driveServo.getDistance();
        double steerRad = m_turningServo.getWrappedPositionRad();
        switch (Identity.instance) {
            case SWERVE_ONE:
            case SWERVE_TWO:
            case COMP_BOT:
                drive_M = correctPositionForSteering(drive_M, steerRad);
                break;
            case BLANK:
            default:
                break;
        }
        return new SwerveModulePosition100(
                drive_M,
                Optional.of(new Rotation2d(steerRad)));
    }

    double turningPosition() {
        return m_turningServo.getWrappedPositionRad();
    }

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
    }

    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }

    /** Update logs. */
    void periodic() {
        m_driveServo.periodic();
        m_turningServo.periodic();
    }

    static double reduceCrossTrackError(double measuredAngleRad, double desiredSpeed, Rotation2d desiredAngle) {
        double error = MathUtil.angleModulus(desiredAngle.getRadians() - measuredAngleRad);
        // cosine is pretty forgiving of misalignment
        // double scale = Math.abs(Math.cos(error));
        // gaussian is much less forgiving. note the adjustable factor. The value of
        // 4 means there is almost no motion past about 60 degrees of error.
        final double width = 4.0;
        double scale = Math.exp(-width * error * error);
        return scale * desiredSpeed;
    }

    /////////////////////////////////////////////////////////////////

    /** Turning servo commands always specify zero-velocity goal. */
    private void actuate(SwerveModuleState100 desired) {
        double speed = desired.speedMetersPerSecond();
        if (desired.angle().isEmpty())
            throw new IllegalArgumentException("actuation needs a real angle");
        Rotation2d desiredAngle = desired.angle().get();

        if (Experiments.instance.enabled(Experiment.CorrectSpeedForSteering)) {
            // help drive motors overcome steering.
            speed = correctSpeedForSteering(speed, desiredAngle);
        }
        if (Experiments.instance.enabled(Experiment.ReduceCrossTrackError)) {
            double measuredAngleRad = m_turningServo.getWrappedPositionRad();
            speed = reduceCrossTrackError(measuredAngleRad, speed, desiredAngle);

        }
        m_driveServo.setVelocity(speed);
        // if (Experiments.instance.enabled(Experiment.UnprofiledSteering)) {
        // no profile, just low-level position
        Control100 control = new Control100(desiredAngle.getRadians(), 0);
        m_turningServo.setPositionDirect(new Setpoints1d(control, control), 0);

        // TODO: finish this
        // } else {
        // // use the profile
        // m_turningServo.setPositionProfiled(desiredAngle.getRadians(), 0);
        // }
        m_previousDesiredAngle = desiredAngle;
    }

    /** Correct the desired speed for steering coupling. */
    private double correctSpeedForSteering(double desiredSpeed, Rotation2d desiredAngle) {
        double dt = dt();
        if (dt > 0.04) {
            // clock is unreliable, don't do anything.
            return desiredSpeed;
        }
        if (dt < 0.01) {
            // avoid short intervals
            return desiredSpeed;
        }
        Rotation2d dtheta = desiredAngle.minus(m_previousDesiredAngle);
        double omega = dtheta.getRadians() / dt;
        // TODO: should this be positive or negative?
        return desiredSpeed + 0.0975 * omega / 3.8;
    }

    /** Correct position measurement for steering coupling. */
    private double correctPositionForSteering(double drive_M, double steerRad) {
        // TODO: replace the magic numbers here with .. i think this is like wheel
        // diameter? radius? gear ratio? what is this?
        // drive is 5.50:1, steer is 10.2, so
        return drive_M - 0.0975 * steerRad / 3.8;
    }

    /**
     * Use the current turning servo position to optimize the desired state.
     * 
     * TODO: should this use unwrapped?
     */
    private SwerveModuleState100 optimize(SwerveModuleState100 desired) {
        return SwerveModuleState100.optimize(
                desired,
                new Rotation2d(m_turningServo.getWrappedPositionRad()));
    }

    /**
     * If the desired angle is empty, replace it with the previous desired angle.
     */
    private SwerveModuleState100 usePreviousAngleIfEmpty(SwerveModuleState100 desired) {
        if (desired.angle().isEmpty()) {
            return new SwerveModuleState100(
                    desired.speedMetersPerSecond(),
                    Optional.of(m_previousDesiredAngle));
        }
        return desired;
    }

    private double dt() {
        double now = Takt.get();
        double dt = now - m_previousTime;
        m_previousTime = now;
        return dt;
    }
}
