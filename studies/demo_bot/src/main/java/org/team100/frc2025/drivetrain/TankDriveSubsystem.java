package org.team100.frc2025.drivetrain;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Two-motor tank drive. */
public class TankDriveSubsystem extends SubsystemBase {
    private static final double ROT_SCALE = 0.3;
    private static final double MAX_SPEED_M_S = 0.4;
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double WHEEL_DIAM = 0.098425;

    private final LinearVelocityServo m_right;
    private final LinearVelocityServo m_left;

    public TankDriveSubsystem(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.type(this);
        m_left = makeServo("Left", log, currentLimit, 3);
        m_right = makeServo("Right", log, currentLimit, 2);
    }

    private static LinearVelocityServo makeServo(
            String name, LoggerFactory log, int currentLimit, int canId) {
        LoggerFactory moduleLogger = log.name(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canId,
                MotorPhase.FORWARD,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants());
        CANSparkEncoder encoder = new CANSparkEncoder(moduleLogger, motor);
        LinearMechanism mech = new LinearMechanism(
                log, motor, encoder, GEAR_RATIO, WHEEL_DIAM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(log, mech);
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
    }

    /**
     * @param translationSpeed -1 to 1
     * @param rotSpeed         -1 to 1
     */
    public void set(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed * ROT_SCALE, false);
        m_left.setVelocity(s.left * MAX_SPEED_M_S);
        m_right.setVelocity(s.right * MAX_SPEED_M_S);
    }

    /**
     * @param translationSpeed -1 to 1
     * @param rotSpeed         -1 to 1
     */
    public void setRaw(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed * ROT_SCALE, false);
        m_left.setVelocity(s.left);
        m_right.setVelocity(s.right);
    }

    public void stop() {
        m_left.stop();
        m_right.stop();
    }
}
