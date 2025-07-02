package org.team100.lib.sensors;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;

/**
 * Produces real or simulated gyros depending on identity.
 */
public class GyroFactory {

    public static Gyro get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        switch (Identity.instance) {
            case SWERVE_ONE:
            case COMP_BOT:
                // return new ReduxGyro(parent, 60);
            default:
                // for simulation
                return new SimulatedGyro(kinodynamics, collection);
        }
    }

    private GyroFactory() {
        //
    }
}
