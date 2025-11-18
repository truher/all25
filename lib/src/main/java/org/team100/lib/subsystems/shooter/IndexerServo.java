package org.team100.lib.subsystems.shooter;

import java.lang.System.Logger;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerServo extends SubsystemBase {

    private final PWM m_servo;
    private final DoubleLogger m_doubleLogger;

    public IndexerServo(LoggerFactory parent, int channel) {
        LoggerFactory logger = parent.type(this);
        m_doubleLogger = logger.doubleLogger(Level.TRACE, "Angle (deg)");
        m_servo = new PWM(channel);
    }

    public void set(double value) {
        m_servo.setSpeed(-1.0 * value);
        m_doubleLogger.log(() -> value);
    }


    public void stop() {
        m_servo.setSpeed(0);
    }

    public void setPosition(double value){
        m_servo.setPosition(value);
    }

    public Command servoOut(){
        return run(() -> setPosition(0.3));
    }

    public Command servoIn(){
        return run(() -> setPosition(0.25));
    }

    public Command feed() {
        return run(() -> set(1));
    }
}
