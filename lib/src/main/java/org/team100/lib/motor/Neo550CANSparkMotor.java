package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Neo550 motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1651/
 */
public class Neo550CANSparkMotor extends CANSparkMotor {
    public Neo550CANSparkMotor(
            LoggerFactory parent,
            int busId,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new SparkMax(busId, canId, MotorType.kBrushless),
                motorPhase, currentLimit, ff, pid);
    }

    @Override
    public double kROhms() {
        return 0.12;
    }

    @Override
    public double kTNm_amp() {
        return 0.009;
    }
}
