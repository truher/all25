package org.team100.frc2025;

import org.team100.lib.examples.tank.BareMotorTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotorGroup;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.TalonSRXMotor;

/** Configuration of motors on the Rookie Bot. */
public class TankFactory {

    public static TankDrive make(LoggerFactory parent, int supplyLimit) {
        LoggerFactory log = parent.name("Tank Drive");
        BareMotorGroup left = new BareMotorGroup(
                new TalonSRXMotor(log.name("left 1"), 5, MotorPhase.REVERSE, supplyLimit),
                new TalonSRXMotor(log.name("left 2"), 11, MotorPhase.REVERSE, supplyLimit));
        BareMotorGroup right = new BareMotorGroup(
                new TalonSRXMotor(log.name("right 1"), 10, MotorPhase.FORWARD, supplyLimit),
                new TalonSRXMotor(log.name("right 2"), 3, MotorPhase.FORWARD, supplyLimit));
        return new BareMotorTank(left, right);
    }

}
