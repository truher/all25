package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.examples.tank.BareMotorTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

/** Configuration of motors on the Rookie Bot. */
public class TankFactory {

    public static TankDrive make(LoggerFactory parent, int supplyLimit) {
        LoggerFactory log = parent.name("Tank Drive");
        System.out.println("Identity" + Identity.instance);
        int rightCanId = switch(Identity.instance) {
            case ROOKIE_BOT -> 1; 
            default -> 1;
        };
        int leftCanId = switch(Identity.instance) {
            case ROOKIE_BOT ->  1;
            default -> 1;
        };

        NeoCANSparkMotor right = new NeoCANSparkMotor(log, new CanId(27), NeutralMode.BRAKE,
         MotorPhase.FORWARD, supplyLimit, Feedforward100.makeNeo(), new PIDConstants());

        NeoCANSparkMotor left = new NeoCANSparkMotor(log, new CanId(3), NeutralMode.BRAKE, 
        MotorPhase.REVERSE, supplyLimit, Feedforward100.makeNeo(), new PIDConstants());
       
        return new BareMotorTank(left, right);
    }
}
