package org.team100.lib.gyro;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A simulated gyro that uses drivetrain odometry.
 */
public class SimulatedGyro implements Gyro {
    private double m_heading = 0;
    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveModuleCollection m_moduleCollection;

    private final DoubleCache m_headingCache;

    private double m_time = Takt.get();

    public SimulatedGyro(
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        m_kinodynamics = kinodynamics;
        m_moduleCollection = collection;
        m_headingCache = Cache.ofDouble(() -> {
            double dt = dt();
            if (dt > 0.04) {
                // clock is unreliable, ignore it
                dt = 0;
            }
            SwerveModuleStates states = m_moduleCollection.states();
            ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
            m_heading += speeds.omegaRadiansPerSecond * dt;
            return m_heading;
        });
    }

    double dt() {
        double now = Takt.get();
        double dt = now - m_time;
        m_time = now;
        return dt;
    }

    @Override
    public Rotation2d getYawNWU() {
        return new Rotation2d(m_headingCache.getAsDouble());
    }

    @Override
    public double getYawRateNWU() {
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        return speeds.omegaRadiansPerSecond;
    }

    @Override
    public Rotation2d getPitchNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public void periodic() {
        //
    }

}
