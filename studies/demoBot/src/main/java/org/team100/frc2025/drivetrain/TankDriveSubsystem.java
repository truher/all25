package org.team100.frc2025.drivetrain;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
    private final OptionalDoubleLogger m_leftlogger;
    private final OptionalDoubleLogger m_rightlogger;
    private final OptionalDoubleLogger m_rearLeftCurrentlogger;
    private final OptionalDoubleLogger m_rearRightCurrentlogger;
    private final OptionalDoubleLogger m_frontLeftCurrentlogger;
    private final OptionalDoubleLogger m_frontRightCurrentlogger;
    private final TankModuleCollection m_modules;
    private final double kMaxSpeedM_S = 0.4;

    public TankDriveSubsystem(
            LoggerFactory parent,
            TankModuleCollection modules) {
        LoggerFactory logger = parent.type(this);
        m_leftlogger = logger.optionalDoubleLogger(Level.TRACE, "Left Drive Velocity M_S");
        m_rightlogger = logger.optionalDoubleLogger(Level.TRACE, "Right Drive Velocity M_S");
        m_rearLeftCurrentlogger = logger.optionalDoubleLogger(Level.TRACE, "Rear Left Drive Current (A)");
        m_rearRightCurrentlogger = logger.optionalDoubleLogger(Level.TRACE, "Rear Right Drive Current (A)");
        m_frontLeftCurrentlogger = logger.optionalDoubleLogger(Level.TRACE, "Front Left Drive Current (A)");
        m_frontRightCurrentlogger = logger.optionalDoubleLogger(Level.TRACE, "Front Right Drive Current (A)");
        m_modules = modules;
    }

    @Override
    public void periodic() {
        m_leftlogger.log(() -> m_modules.getSpeeds()[0]);
        m_rightlogger.log(() -> m_modules.getSpeeds()[1]);
        m_rearLeftCurrentlogger.log(() -> m_modules.getCurrent()[0]);
        m_rearRightCurrentlogger.log(() -> m_modules.getCurrent()[1]);
        m_frontLeftCurrentlogger.log(() -> m_modules.getCurrent()[2]);
        m_frontRightCurrentlogger.log(() -> m_modules.getCurrent()[3]);
    }

    /**
     * 
     * @param translationSpeed -1 to 1 duty cycle
     * @param rotSpeed         -1 to 1 duty cycle
     * 
     */
    public void set(double translationSpeed, double rotSpeed) {
        double invertedRot = rotSpeed * -1.0;
        double leftSpeed = translationSpeed + invertedRot;
        double rightSpeed = translationSpeed - invertedRot;
        setModules(leftSpeed * kMaxSpeedM_S, rightSpeed * kMaxSpeedM_S);
    }

    /**
     * 
     * @param translationSpeed -1 to 1 duty cycle
     * @param rotSpeed         -1 to 1 duty cycle
     * 
     */
    public void setRaw(double translationSpeed, double rotSpeed) {
        double invertedRot = rotSpeed * -1.0;
        double leftSpeed = translationSpeed + invertedRot;
        double rightSpeed = translationSpeed - invertedRot;
        setModules(leftSpeed, rightSpeed);
    }

    public void setModules(double... states) {
        m_modules.setDrive(states);
    }

    public void stop() {
        m_modules.stop();
    }
}
