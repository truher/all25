package org.team100.lib.sensor.gyro;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.util.CanId;

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
                return new ReduxGyro(parent, new CanId(60));
            default:
                // for simulation
                return new SimulatedGyro(parent, kinodynamics, collection);
        }
    }

    private GyroFactory() {
        //
    }
}
