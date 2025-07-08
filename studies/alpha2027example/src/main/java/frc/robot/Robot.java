package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    // private final LSM6DSOXGyro m_gyro;
    // private final AS5048B_I2C m_encoder;
    // private final BNO080 m_imu;
    private final APDS9960 m_sensor;
    // private final DoublePublisher yawPub;
    // private final DoublePublisher anglePub;
    // private final DoublePublisher imuPub;
    private final DoublePublisher sensorPub;

    public Robot() {
        // m_gyro = new LSM6DSOXGyro();
        // m_encoder = new AS5048B_I2C();
        // m_imu = new BNO080();
        m_sensor = new APDS9960();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // NetworkTable gyroTable = inst.getTable("gyro");
        // yawPub = gyroTable.getDoubleTopic("yaw (rad)").publish();
        // NetworkTable encoderTable = inst.getTable("encoder");
        // anglePub = encoderTable.getDoubleTopic("angle").publish();
        // NetworkTable imuTable = inst.getTable("imu");
        // imuPub = imuTable.getDoubleTopic("angle").publish();
        NetworkTable sensorTable = inst.getTable("sensor");
        sensorPub = sensorTable.getDoubleTopic("level").publish();
    }

    @Override
    public void teleopInit() {
        System.out.println("teleop init");
        // m_imu.begin();
    }

    @Override
    public void teleopPeriodic() {
        // System.out.println("teleop periodic");
        // yawPub.set(m_gyro.getYawNWU().getRadians());
        // anglePub.set(m_encoder.get());

        ////////////////////////////////
        ///
        // Optional<Double> gyroRotation = m_imu.getYaw();
        // if (gyroRotation.isEmpty()) {
        // // System.out.println("null gyro rotation");
        // } else {
        // // System.out.printf("gyro rotation %s", gyroRotation);
        // imuPub.set(gyroRotation.get());
        // }

        int val = m_sensor.read();
        sensorPub.set(val);
    }

    @Override
    public void testPeriodic() {
        // Optional<Double> gyroRotation = m_imu.getYaw();
    }
}
