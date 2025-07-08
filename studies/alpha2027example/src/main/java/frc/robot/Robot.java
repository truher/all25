package frc.robot;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    // private final LSM6DSOXGyro m_gyro;
    // private final AS5048B_I2C m_encoder;
    // private final BNO080 m_imu;
    // private final APDS9960 m_sensor;
    private final BNO086_I2C myIMU;

    // private final DoublePublisher yawPub;
    // private final DoublePublisher anglePub;
    private final DoublePublisher rollPub;
    private final DoublePublisher pitchPub;
    private final DoublePublisher yawPub;
    private final DoublePublisher accuracyPub;
    // private final DoublePublisher xPub;
    // private final DoublePublisher yPub;
    // private final DoublePublisher zPub;
    // private final DoublePublisher sensorPub;

    private byte counter;
    private final Timer timer;

    private final IntegerPublisher counterPub;
    private final DoublePublisher loopPub;

    public Robot() {
        // m_gyro = new LSM6DSOXGyro();
        // m_encoder = new AS5048B_I2C();
        // m_imu = new BNO080();
        // m_sensor = new APDS9960();
        myIMU = new BNO086_I2C();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // NetworkTable gyroTable = inst.getTable("gyro");
        // yawPub = gyroTable.getDoubleTopic("yaw (rad)").publish();
        // NetworkTable encoderTable = inst.getTable("encoder");
        // anglePub = encoderTable.getDoubleTopic("angle").publish();
        NetworkTable imuTable = inst.getTable("imu");
        rollPub = imuTable.getDoubleTopic("roll").publish();
        pitchPub = imuTable.getDoubleTopic("pitch").publish();
        yawPub = imuTable.getDoubleTopic("yaw").publish();
        accuracyPub = imuTable.getDoubleTopic("accuracy").publish();
        // xPub = imuTable.getDoubleTopic("x").publish();
        // yPub = imuTable.getDoubleTopic("y").publish();
        // zPub = imuTable.getDoubleTopic("z").publish();
        // NetworkTable sensorTable = inst.getTable("sensor");
        // sensorPub = sensorTable.getDoubleTopic("level").publish();
        counterPub = inst.getTable("counter").getIntegerTopic("value").publish();
        loopPub = inst.getTable("timer").getDoubleTopic("value").publish();
        timer = new Timer();
    }

    @Override
    public void teleopInit() {
        System.out.println("teleop init begin");
        myIMU.begin();
        System.out.println("teleop init enable rotation");
        myIMU.enableRotationVector();
        // System.out.println("teleop init enable accel");
        // myIMU.enableAccelerometer();
        // myIMU.enableUncalibratedGyro();
        System.out.println("teleop init done");
        counter = 0;
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

        // int val = m_sensor.read();
        // sensorPub.set(val);
    }

    @Override
    public void testInit() {
        timer.start();
    }

    @Override
    public void testPeriodic() {
        counter++;
        // counterPub.set(counter);

        // I dunno what this represents.  delta to the interrupt?
        // int timestamp = myIMU.getTimeStamp();

        // double t = timer.get();
        // loopPub.set(t);
        // timer.reset();


        // Optional<Double> gyroRotation = m_imu.getYaw();

        if (myIMU.dataAvailable()) {
            // float quatI = myIMU.getQuatI();
            // float x = myIMU.getAccelX();
            // System.out.printf("quatI %f\n", quatI);
            // System.out.printf("accelX %f\n", x);

            Quaternion q = myIMU.getQuaternion();
            double roll = new Rotation3d(q).getX();
            double pitch = new Rotation3d(q).getY();
            double yaw = new Rotation3d(q).getZ();
            double accuracy = myIMU.getQuatRadianAccuracy();

            System.out.printf("roll %f pitch %f yaw %f\n", roll, pitch, yaw);
            rollPub.set(roll);
            pitchPub.set(pitch);
            yawPub.set(yaw);
            accuracyPub.set(accuracy);

            // Vector<N3> gyro = myIMU.getUncalibratedGyro();

            // Vector<N3> acc = myIMU.getAccel();
            // xPub.set(acc.get(0));
            // yPub.set(acc.get(1));
            // zPub.set(acc.get(2));
            // System.out.printf("x %f y %f z %f\n", acc.get(0), acc.get(1), acc.get(2));
        }
    }
}
