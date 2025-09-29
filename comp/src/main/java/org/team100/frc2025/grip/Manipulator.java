package org.team100.frc2025.grip;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.LazySimulatedBareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.SimulatedBareMotor;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

    private static final int NEAR = 100;
    private final BareMotor m_algaeMotor;
    private final LinearMechanism m_leftMech;
    private final LinearMechanism m_rightMech;
    private final LinearMechanism m_algaeMech;
    private final LaserCanInterface m_rightLaser;
    private final LaserCanInterface m_frontLaser;
    private final LaserCanInterface m_backLaser;
    private final LaserCanInterface m_leftLaser;

    public Manipulator(LoggerFactory log) {
        switch (Identity.instance) {
            case COMP_BOT -> {
                // Set specific parameters for the competition robot               //can id done
                Kraken6Motor leftMotor = new Kraken6Motor(log, 19, NeutralMode.COAST, MotorPhase.FORWARD, 
                10, //og 40
                10, //og 40
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                Kraken6Motor rightMotor = new Kraken6Motor(log, 20, NeutralMode.COAST, MotorPhase.REVERSE,  //can id done
                10, //og 40
                10, //og 40
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                Kraken6Motor algaeMotor = new Kraken6Motor(log, 21, NeutralMode.COAST, MotorPhase.FORWARD,  //can id done
                10, //og 120
                10, //og 120
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                m_algaeMotor = algaeMotor;
                m_rightLaser = new LaserCan(17); //can id done
                m_frontLaser = new LaserCan(16); //can id done
                m_backLaser = new LaserCan(18); //can id done
                m_leftLaser = new LaserCan(15); //can id done
                m_leftMech = new LinearMechanism(log, leftMotor, new Talon6Encoder(log, leftMotor), 16,
                        .1, -100000000, 1000000);
                m_rightMech = new LinearMechanism(log, rightMotor, new Talon6Encoder(log, rightMotor),
                        16, .1, -100000000, 1000000);
                m_algaeMech = new LinearMechanism(log, algaeMotor, new Talon6Encoder(log, algaeMotor),
                        16, .1, -100000000, 1000000);
            }
            default -> {
                BareMotor leftMotor = new SimulatedBareMotor(log, 100);
                SimulatedBareEncoder leftEncoder = new SimulatedBareEncoder(log, leftMotor);
                BareMotor rightMotor = new SimulatedBareMotor(log, 100);
                SimulatedBareEncoder rightEncoder = new SimulatedBareEncoder(log, rightMotor);
                // simulated algae motor gets overloaded 2 sec after starting
                BareMotor algaeMotor = new LazySimulatedBareMotor(
                        new SimulatedBareMotor(log, 100), 2);
                SimulatedBareEncoder algaeEncoder = new SimulatedBareEncoder(log, algaeMotor);
                m_algaeMotor = algaeMotor;
                m_leftMech = new LinearMechanism(
                        log, leftMotor, leftEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMech = new LinearMechanism(
                        log, rightMotor, rightEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMech = new LinearMechanism(
                        log, algaeMotor, algaeEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightLaser = new MockLaserCan();
                m_frontLaser = new MockLaserCan();
                m_backLaser = new MockLaserCan();
                m_leftLaser = new MockLaserCan();
            }
        }
    }

    public boolean isCoralClose(LaserCanInterface sensor) {
        Measurement m = sensor.getMeasurement();
        if (m == null)
            return false;
        return m.distance_mm < NEAR;
    }

    public void intakeCenter() {
        if (hasCoral()) {
            stopMotors();
        } else {
            m_algaeMech.setDutyCycle(-0.5);
            m_leftMech.setDutyCycle(0.5);
            m_rightMech.setDutyCycle(0.5);
        }
    }

    public boolean hasCoral() {
        return isCoralClose(m_backLaser);
    }

    public void ejectCenter() {
        m_algaeMech.setDutyCycle(0.5);
        m_leftMech.setDutyCycle(-0.5);
        m_rightMech.setDutyCycle(-0.5);
    }

    public void intakeSideways() {
        if (hasCoralSideways()) {
            stopMotors();
        } else {
            m_algaeMech.setDutyCycle(-0.5);
            if (isCoralClose(m_leftLaser)) {
                m_leftMech.setDutyCycle(0.5);
                m_rightMech.setDutyCycle(-0.5);
            } else {
                m_leftMech.setDutyCycle(-0.5);
                m_rightMech.setDutyCycle(0.5);
            }
        }
    }

    public boolean hasCoralSideways() {
        return isCoralClose(m_leftLaser) && isCoralClose(m_rightLaser);
    }

    public void stopMotors() {
        m_algaeMech.setDutyCycle(0);
        m_leftMech.setDutyCycle(0);
        m_rightMech.setDutyCycle(0);
    }

    /**
     * Current is high when the algae is in.
     * (...and also at startup so include a delay.)
     */
    public boolean hasAlgae() {
        return m_algaeMotor.getCurrent() > 80;
    }

    /////////////////////////////////////////////////
    //
    // COMMANDS

    /** This is not "hold position" it is "disable". */
    public Command stop() {
        return startRun(this::lowAlgaeTorque, this::stopMotors);
    }

    public Command algaeHold() {
        return startRun(this::lowAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeIntake() {
        return startRun(this::highAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeEject() {
        return startRun(this::highAlgaeTorque, this::ejectAlgae);
    }

    public Command centerIntake() {
        return run(this::intakeCenter);
    }

    public Command centerEject() {
        return run(this::ejectCenter);
    }

    //////////////////////////////////////////////////

    /**
     * Set high current limits.
     * Previous grip used 90A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 1.62 Nm.
     */
    private void highAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(1.65);
    }

    /**
     * Set moderate current limits.
     * Previous grip used 35A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 0.63 Nm.
     */
    private void lowAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(0.65);
    }

    private void intakeAlgae() {
        m_algaeMech.setDutyCycle(1);
    }

    public void ejectAlgae() {
        m_algaeMech.setDutyCycle(-0.1);
    }
}
