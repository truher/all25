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
public class RealisticFixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveModelHistory history;
    public SwerveModelEstimate estimate;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public SwerveController controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public RealisticFixture() {
        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        history = new SwerveModelHistory(
                logger,
                swerveKinodynamics);
        estimate = new SwerveModelEstimate(history);
        OdometryUpdater ou = new OdometryUpdater(swerveKinodynamics, gyro, history, collection::positions);
        ou.reset(Pose2d.kZero, 0);
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
