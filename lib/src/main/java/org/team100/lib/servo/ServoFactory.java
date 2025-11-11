package org.team100.lib.servo;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.sensor.position.incremental.sim.SimulatedBareEncoder;

public class ServoFactory {
    /** An OutboardLinearVelocityServo with simulated motor and encoder */
    public static OutboardLinearVelocityServo simulatedOutboardLinearVelocityServo(
            LoggerFactory log,
            double gearRatio,
            double wheelDia) {
        BareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        return new OutboardLinearVelocityServo(
                log, new LinearMechanism(
                        log, motor, encoder, gearRatio, wheelDia,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
    }

}
