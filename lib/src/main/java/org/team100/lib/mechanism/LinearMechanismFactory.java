package org.team100.lib.mechanism;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.incremental.rev.CANSparkEncoder;
import org.team100.lib.sensor.position.incremental.sim.SimulatedBareEncoder;
import org.team100.lib.util.CanId;

/**
 * Factory for common mechanisms.
 */
public class LinearMechanismFactory {
    private final int m_statorCurrentLimit;
    private final double m_gearRatio;
    private final double m_wheelDiaM;

    /** For multiples, they always have the same gear ratios and current limits. */
    public LinearMechanismFactory(int statorCurrentLimit, double gearRatio, double wheelDiaM) {
        m_statorCurrentLimit = statorCurrentLimit;
        m_gearRatio = gearRatio;
        m_wheelDiaM = wheelDiaM;
    }

    /** REV NEO motor with built-in encoder. */
    public LinearMechanism neo(LoggerFactory log, CanId id, MotorPhase phase) {
        Feedforward100 ff = Feedforward100.makeNeo(log);
        // 10/22/25: Anay found this value worked well
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.00005);//0.00005
        // 10/22/25: Lincoln used this value
        // PIDConstants.makeVelocityPID(0.0003));
        NeoCANSparkMotor motor = new NeoCANSparkMotor(
                log, id, NeutralMode.BRAKE, phase, m_statorCurrentLimit, ff, pid);
        CANSparkEncoder encoder = new CANSparkEncoder(log, motor);
        return new LinearMechanism(log, motor, encoder, m_gearRatio, m_wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /** REV NEO 550 motor with built-in encoder. */
    public LinearMechanism neo550(LoggerFactory log, CanId id, MotorPhase phase) {
        Feedforward100 ff = Feedforward100.makeNeo550(log);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0001);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                log, id, NeutralMode.BRAKE, phase, m_statorCurrentLimit, ff, pid);
        CANSparkEncoder encoder = new CANSparkEncoder(log, motor);
        return new LinearMechanism(log, motor, encoder, m_gearRatio, m_wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /** Simulation. */
    public LinearMechanism sim(LoggerFactory log) {
        BareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        return new LinearMechanism(log, motor, encoder, m_gearRatio, m_wheelDiaM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }
}
