package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final LSM6DSOXGyro m_gyro;
    private final AS5048B_I2C m_encoder;
    private final DoublePublisher yawPub;
    private final DoublePublisher anglePub;

    public Robot() {
        m_gyro = new LSM6DSOXGyro();
        m_encoder = new AS5048B_I2C();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable gyroTable = inst.getTable("gyro");
        yawPub = gyroTable.getDoubleTopic("yaw (rad)").publish();
        NetworkTable encoderTable = inst.getTable("encoder");
        anglePub = encoderTable.getDoubleTopic("angle").publish();
    }

    @Override
    public void teleopPeriodic() {
        yawPub.set(m_gyro.getYawNWU().getRadians());
        anglePub.set(m_encoder.get());
    }
}
