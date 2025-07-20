package org.team100.five_bar.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplest possible control of the five-bar.
 */
public class FiveBarBare extends SubsystemBase {

    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;

    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_motorP1;

    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_motorP5;

    public FiveBarBare(LoggerFactory logger) {
        // zeros
        PIDConstants pid = new PIDConstants();
        Feedforward100 ff = Feedforward100.zero();

        LoggerFactory loggerP1 = logger.child("p1");
        m_motorP1 = new Falcon6Motor(
                loggerP1,
                1,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);

        LoggerFactory loggerP5 = logger.child("p5");
        m_motorP5 = new Falcon6Motor(
                loggerP5,
                2,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
    }

    /////////////////////

    private void setDutyCycle(double p1, double p5) {
        m_motorP1.setDutyCycle(p1);
        m_motorP5.setDutyCycle(p5);
    }

    /////////////////////
    // 
    // Commands

    public Command dutyCycle(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setDutyCycle(p1.getAsDouble(), p5.getAsDouble()));
    }
}
