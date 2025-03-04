// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Elevator;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Glassy {
    /** Creates a new Elevator. */

    private static final double kElevatorReduction = 2; // TODO CHANGE THIS
    private static final double kElevatorWheelDiamater = 1; // TODO CHANGE THIS

    private static final double kWristReduction = 1; // TODO CHANGE THIS
    private static final double kWristWheelDiameter = 1; // TODO CHANGE THIS

    private final OutboardLinearPositionServo starboardServo;
    private final OutboardLinearPositionServo portServo;

    private final ElevatorVisualization m_viz;

    public Elevator(
            LoggerFactory parent,
            int starboardID,
            int portID) {

        LoggerFactory child = parent.child(this);

        LoggerFactory starboardLogger = child.child("Starboard");
        LoggerFactory portLogger = child.child("Port");
        LoggerFactory wristLogger = child.child("Wrist");

        LoggerFactory starboardMotorLogger = child.child("Starboard Motor");
        LoggerFactory portMotorLogger = child.child("Port Motor");
        LoggerFactory wristMotorLogger = child.child("Wrist Motor");

        int elevatorSupplyLimit = 60;
        int elevatorStatorLimit = 90;

        // PIDConstants elevatorPID = new PIDConstants(2, 0, 0);
        PIDConstants elevatorPID = PIDConstants.makePositionPID(2);

        Feedforward100 elevatorFF = Feedforward100.makeKraken6Elevator();
        // TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(220, 220,
        // 0.05); // TODO CHANGE THESE
        TrapezoidProfile100 elevatorProfile = new TrapezoidProfile100(100, 100, 0.05); // TODO CHANGE THESE

        int wristSupplyLimit = 60;
        int wristStatorLimit = 90;

        PIDConstants wristPID = PIDConstants.makePositionPID(10);

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();
        PIDController wristPIDController = new PIDController(wristPID.getPositionP(), wristPID.getPositionI(),
                wristPID.getPositionD());
        wristPIDController.setTolerance(0.02);
        wristPIDController.setIntegratorRange(0, 1);
        TrapezoidProfile100 wristProfile = new TrapezoidProfile100(20, 20, 0.05); // TODO CHANGE THESE

        switch (Identity.instance) {
            case FRC_100_ea4 -> {
                Kraken6Motor starboardMotor = new Kraken6Motor(starboardMotorLogger, starboardID, MotorPhase.FORWARD,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);
                Kraken6Motor portMotor = new Kraken6Motor(portMotorLogger, portID, MotorPhase.REVERSE,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);

                LinearMechanism starboardMech = new SimpleLinearMechanism(
                        starboardMotor,
                        new Talon6Encoder(starboardLogger, starboardMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);

                starboardServo = new OutboardLinearPositionServo(starboardLogger, starboardMech, elevatorProfile);

                LinearMechanism portMech = new SimpleLinearMechanism(
                        portMotor,
                        new Talon6Encoder(portLogger, portMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);

                portServo = new OutboardLinearPositionServo(portLogger, portMech, elevatorProfile);

                break;
            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(starboardMotorLogger, 100);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(portMotorLogger, 100);

                LinearMechanism starboardMech = new SimpleLinearMechanism(
                        starboardMotor,
                        new SimulatedBareEncoder(starboardLogger, starboardMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);
                LinearMechanism portMech = new SimpleLinearMechanism(
                        portMotor,
                        new SimulatedBareEncoder(portLogger, portMotor),
                        kElevatorReduction,
                        kElevatorWheelDiamater);
                starboardServo = new OutboardLinearPositionServo(starboardLogger, starboardMech, elevatorProfile);
                portServo = new OutboardLinearPositionServo(portLogger, portMech, elevatorProfile);
            }

        }
        m_viz = new ElevatorVisualization(this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        starboardServo.periodic();
        portServo.periodic();
        m_viz.viz();
    }

    public void resetElevatorProfile() {
        starboardServo.reset();
        portServo.reset();
    }

    public void resetWristProfile() {
    }

    /**
     * TODO: calibrate this in meters
     */
    public void setPosition(double x) {
        // TODO: change gravity depending on position (it gets heavier)
        starboardServo.setPosition(x, 1.3); // 54 max
        portServo.setPosition(x, 1.3); // 54 max
    }

    public void setDutyCycle(double value) {
        // starboardServo.setPosition(30, 2); //54 max
        // portServo.setPosition(30, 2); //54 max
        starboardServo.setDutyCycle(value);
        portServo.setDutyCycle(value);
    }

    /**
     * TODO: calibrate this in meters
     */
    public double getPosition() {
        return starboardServo.getPosition().orElse(0);
    }

}
