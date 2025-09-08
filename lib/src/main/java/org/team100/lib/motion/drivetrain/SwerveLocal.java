package org.team100.lib.motion.drivetrain;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModulePosition100Logger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.state.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.state.SwerveModuleStates;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 * 
 * Most methods in this class should be package-private, they're only used by
 * SwerveDriveSubsystem, and by tests.
 */
public class SwerveLocal {
    private static final boolean DEBUG = false;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final SwerveModulePosition100Logger m_swerveModuleFrontLeftPosition;
    private final SwerveModulePosition100Logger m_swerveModuleFrontRightPosition;
    private final SwerveModulePosition100Logger m_swerveModuleRearLeftPosition;
    private final SwerveModulePosition100Logger m_swerveModuleRearRightPosition;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.type(this);
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed");
        m_swerveModuleFrontLeftPosition = child.swerveModulePosition100Logger(Level.TRACE, "Fromt Left");
        m_swerveModuleFrontRightPosition = child.swerveModulePosition100Logger(Level.TRACE, "Front Right");
        m_swerveModuleRearLeftPosition = child.swerveModulePosition100Logger(Level.TRACE, "Rear Left");
        m_swerveModuleRearRightPosition = child.swerveModulePosition100Logger(Level.TRACE, "Rear Right");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

    /**
     * Discretizes the speeds, calculates the inverse kinematic module states, and
     * sets the module states.
     */
    void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds);
        setModuleStates(states);
        m_log_chassis_speed.log(() -> speeds);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    void defense() {
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(SwerveModuleStates.statesX);
    }

    /**
     * Sets wheel rotation to zero, for optimizing steering control.
     */
    void steer0() {
        setRawModuleStates(SwerveModuleStates.states0);
    }

    /**
     * Sets wheel rotation to 90 degrees, for optimizing steering control.
     */
    void steer90() {
        setRawModuleStates(SwerveModuleStates.states90);
    }

    void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * 
     * Works fine with empty angles.
     * 
     * This "raw" mode is just for testing.
     */
    void setRawModuleStates(SwerveModuleStates targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    SwerveModulePositions positions() {
        return m_modules.positions();
    }

    Translation2d[] getModuleLocations() {
        return m_swerveKinodynamics.getKinematics().getModuleLocations();
    }

    boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    ///////////////////////////////////////////

    void close() {
        m_modules.close();
    }

    void reset() {
        if (DEBUG)
            Util.warn("make sure resetting in SwerveLocal doesn't break anything");
        m_modules.reset();
    }

    /** Updates visualization. */
    void periodic() {
        m_swerveModuleFrontLeftPosition.log(() -> positions().frontLeft());
        m_swerveModuleFrontRightPosition.log(() -> positions().frontRight());
        m_swerveModuleRearLeftPosition.log(() -> positions().rearLeft());
        m_swerveModuleRearRightPosition.log(() -> positions().rearRight());
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleStates states) {
        m_modules.setDesiredStates(states);
    }
}
