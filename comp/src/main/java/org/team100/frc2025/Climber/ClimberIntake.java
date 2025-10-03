package org.team100.frc2025.Climber;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.LazySimulatedBareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIntake extends SubsystemBase {

    private final BareMotor m_motor;
    private double desiredSpeed = 0;
    private int timer;
    private boolean climbReady;

    public ClimberIntake(LoggerFactory parent, CanId canID) {
        LoggerFactory log = parent.type(this);
        climbReady = false;
        timer = 0;
        switch (Identity.instance) {
            case COMP_BOT -> {
                m_motor = new Kraken6Motor(
                        log, canID, NeutralMode.COAST, MotorPhase.REVERSE,
                        20, // og 50
                        20, // og 2
                        new PIDConstants(),
                        Feedforward100.makeKrakenClimberIntake());
            }
            default -> {
                m_motor = new LazySimulatedBareMotor(
                        new SimulatedBareMotor(log, 100), 1.5);
            }
        }
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public boolean isSlow() {
        timer++;
        if (desiredSpeed == 1 && m_motor.getVelocityRad_S() < 1 && timer > 50) {
            climbReady = true;
        }
        return m_motor.getVelocityRad_S() < 1;
    }

    public boolean isClimbReady() {
        return climbReady;
    }

    // COMMANDS

    public Command stop() {
        return run(this::stopMotor);
    }

    public Command intake() {
        return run(this::fullSpeed);
    }

    ////////////////

    private void stopMotor() {
        desiredSpeed = 0;
        m_motor.setDutyCycle(0);
    }

    private void fullSpeed() {
        desiredSpeed = 1;
        m_motor.setDutyCycle(1);
    }

}
