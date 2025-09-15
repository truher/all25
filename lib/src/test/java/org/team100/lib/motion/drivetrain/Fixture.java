package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.SimulatedGyro;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveModelEstimate;
import org.team100.lib.localization.SwerveModelHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 * 
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class Fixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveModelHistory history;
    public SwerveModelEstimate estimate;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public OdometryUpdater ou;
    public SwerveDriveSubsystem drive;
    public SwerveController controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public Fixture() {
        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        history = new SwerveModelHistory(
                logger,
                swerveKinodynamics); // initial time is zero here for testing
        history.reset(gyro.getYawNWU(), collection.positions(), Pose2d.kZero, 0);
        estimate = new SwerveModelEstimate(history);
        ou = new OdometryUpdater(swerveKinodynamics, history, collection::positions);

        SwerveLimiter limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);
        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                gyro,
                swerveKinodynamics,
                ou,
                history,
                swerveLocal,
                () -> {
                },
                limiter);

        controller = SwerveControllerFactory.test(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
