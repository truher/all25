package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final LSM6DSOXGyro m_gyro;
    private final DoublePublisher yawPub;

    public Robot() {
        m_gyro = new LSM6DSOXGyro();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable gyroTable = inst.getTable("gyro");
        yawPub = gyroTable.getDoubleTopic("yaw (rad)").publish();
    }

    @Override
    public void teleopPeriodic() {
        yawPub.set(m_gyro.getYawNWU().getRadians());
    }
}
