package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.examples.tank.BareMotorTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotorGroup;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.TalonSRXMotor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Configuration of motors on the Rookie Bot. */
public class TankFactory {

    public static TankDrive make(LoggerFactory parent, int supplyLimit) {
        LoggerFactory log = parent.name("Tank Drive");

        NeoCANSparkMotor right = new NeoCANSparkMotor(log, 27, MotorPhase.FORWARD, supplyLimit, Feedforward100.makeNeo(), new PIDConstants());

        NeoCANSparkMotor left = new NeoCANSparkMotor(log, 3, MotorPhase.REVERSE, supplyLimit, Feedforward100.makeNeo(), new PIDConstants());
       
        return new BareMotorTank(left, right);
    }
}
