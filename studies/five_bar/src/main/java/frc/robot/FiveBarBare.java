package frc.robot;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct motor control. We wouldn't do it this way for comp, but it's good for
 * learning.
 */
public class FiveBarBare extends SubsystemBase {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;

    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_p1;
    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_p5;

    public FiveBarBare(LoggerFactory logger) {
        // zeros
        PIDConstants pid = new PIDConstants();
        Feedforward100 ff = Feedforward100.zero();
        m_p1 = new Falcon6Motor(
                logger.child("p1"),
                1,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        m_p5 = new Falcon6Motor(
                logger.child("p5"),
                2,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
    }
}
