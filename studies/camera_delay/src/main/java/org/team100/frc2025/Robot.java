package org.team100.frc2025;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.network.RawTags;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot100 {
    /**
     * The position sensor is assumed to have a fixed delay of 600 us.
     */
    private static final double SENSOR_DELAY_S = 0.0006;
    /**
     * The camera delay should be zero because of how NetworkTables handles
     * timestamps.
     */
    private static final double CAMERA_DELAY_S = 0.000;

    private final BareMotor m_motor;

    private final RotaryPositionSensor m_sensor;
    private final RawTags m_rawTags;

    private final TimeInterpolatableBuffer100<Rotation2d> m_sensorBuffer;
    private final TimeInterpolatableBuffer100<Rotation2d> m_cameraBuffer;

    private final DoubleLogger m_logSensor;
    private final DoubleLogger m_logCamera;

    private final SimulatedAS5048 m_simSensor;
    private final SimulatedCamera m_simCamera;

    public Robot() {
        Logging logging = Logging.instance();
        LoggerFactory log = logging.rootLogger;

        m_motor = NeoVortexCANSparkMotor.get(
                log,
                new CanId(1),
                MotorPhase.FORWARD,
                1, // current limit
                Feedforward100.makeNeoVortex(log),
                PIDConstants.makeVelocityPID(log, 0.0002));

        RoboRioChannel sensorChannel = new RoboRioChannel(1);
        m_sensor = new AS5048RotaryPositionSensor(
                log,
                sensorChannel,
                0.0, // offset
                EncoderDrive.DIRECT);

        m_rawTags = new RawTags(log, this::acceptTag);

        m_sensorBuffer = new TimeInterpolatableBuffer100<>(2, 0, Rotation2d.kZero);
        m_cameraBuffer = new TimeInterpolatableBuffer100<>(2, 0, Rotation2d.kZero);

        m_logSensor = log.doubleLogger(Level.TRACE, "sensor");
        m_logCamera = log.doubleLogger(Level.TRACE, "camera");

        if (RobotBase.isSimulation()) {
            // setup simulated data sources
            m_simSensor = new SimulatedAS5048(sensorChannel);
            m_simCamera = new SimulatedCamera();
        } else {
            m_simSensor = null;
            m_simCamera = null;
        }
    }

    private void acceptTag(Transform3d t, double timestamp) {
        Rotation2d cameraRotation = new Rotation2d(t.getRotation().getX());
        m_cameraBuffer.put(timestamp - CAMERA_DELAY_S, cameraRotation);
    }

    @Override
    public void robotPeriodic() {

        Takt.update();
        Cache.refresh();

        if (RobotBase.isSimulation()) {
            // update the simulated inputs
            m_simSensor.run();
            m_simCamera.run();
        }


        double now = Takt.actual();

        // read the sensor and update the sensor buffer
        Rotation2d sensorRotation = new Rotation2d(
                m_sensor.getWrappedPositionRad());
        m_sensorBuffer.put(now - SENSOR_DELAY_S, sensorRotation);

        // read the camera and update the camera buffer
        m_rawTags.update();

        // sample all the buffers from 1 sec ago and log
        double past = now - 1.0;
        m_logSensor.log(() -> m_sensorBuffer.get(past).getRadians());
        m_logCamera.log(() -> m_cameraBuffer.get(past).getRadians());
    }

    @Override
    public void teleopInit() {
        m_motor.setVelocity(1, 0, 0);
    }

    @Override
    public void teleopExit() {
        m_motor.stop();
    }

}
