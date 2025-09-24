package org.team100.frc2025.Climber;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static org.team100.lib.hid.ControlUtil.deadband;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIntake extends SubsystemBase {

    private final BareMotor m_motor;

    public ClimberIntake(LoggerFactory parent, int canID) {
        LoggerFactory log = parent.name("ClimberIntake");

        switch (Identity.instance) {
            case COMP_BOT -> {
                // add the X44 Kraken motor for the active intake
                SimulatedBareMotor intakeMotor = new SimulatedBareMotor(log, 100);

                m_motor = intakeMotor;
            }

            default -> {
                // add the X44 Kraken motor for the active intake
                SimulatedBareMotor intakeMotor = new SimulatedBareMotor(log, 100);

                m_motor = intakeMotor;
            }
        }
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    } 

    public double getVelocityRad_S(){
        return m_motor.getVelocityRad_S();
    }

    // COMMANDS

    public Command stop() {
        return run(
                () -> m_motor.setDutyCycle(0));
    }

    public Command startIntake() {
        return run(
                () -> m_motor.setDutyCycle(1));
    }
}
