package org.team100.frc2025.Funnel;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    private final LinearMechanism m_starboardMech;
    private final LinearMechanism m_portMech;
    private Servo latchingServo1;
    private Servo latchingServo2;
    private DoubleLogger servoAngle1Log;
    private DoubleLogger servoAngle2Log;

    public Funnel(
            LoggerFactory parent,
            int starboardID,
            int portID) {
        LoggerFactory child = parent.type(this);

        switch (Identity.instance) {
            case COMP_BOT -> {
                int funnelSupplyLimit = 20;
                m_starboardMech = Neo550Factory.getNEO550LinearMechanism(
                        child.name("starboard"), funnelSupplyLimit, starboardID, 1,
                        MotorPhase.REVERSE, 1);
                m_portMech = Neo550Factory.getNEO550LinearMechanism(
                        child.name("port"), funnelSupplyLimit, portID, 1,
                        MotorPhase.FORWARD, 1);

                latchingServo1 = new Servo(3);
                latchingServo2 = new Servo(9);

                servoAngle1Log = child.doubleLogger(Level.TRACE, "Servo 1 Angle");
                servoAngle2Log = child.doubleLogger(Level.TRACE, "Servo 2 Angle");
                break;
            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(child, 100);
                SimulatedBareEncoder starboardEncoder = new SimulatedBareEncoder(child, starboardMotor);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(child, 100);
                SimulatedBareEncoder portEncoder = new SimulatedBareEncoder(child, portMotor);
                m_starboardMech = new LinearMechanism(
                        child, starboardMotor, starboardEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_portMech = new LinearMechanism(
                        child, portMotor, portEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
        }
    }

    public void setStarboard(double value) {
        m_starboardMech.setDutyCycle(value);
    }

    public void setPort(double value) {
        m_portMech.setDutyCycle(value);
    }

    public void setFunnel(double value) {
        setStarboard(value);
        setPort(value);
    }

    public void setLatch1(double value) {
        if (latchingServo1 == null)
            return;
        latchingServo1.setAngle(value);
    }

    public double getLatch1() {
        return latchingServo1.getAngle();
    }

    public void setLatch2(double value) {
        if (latchingServo2 == null)
            return;
        latchingServo2.setAngle(value);
    }

    public double getLatch2() {
        return latchingServo2.getAngle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (servoAngle1Log == null)
            return;
        servoAngle1Log.log(() -> getLatch1());
        servoAngle2Log.log(() -> getLatch2());
    }

    // COMMANDS

    public Command setLatch(double v1, double v2) {
        return run(() -> {
            setLatch1(v1);
            setLatch2(v2);
        });
    }

    public Command agitate() {
        return new RunFunnel(this);
    }

}
