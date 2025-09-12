package org.team100.frc2025.Elevator;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    /**
     * Publish the elevator mechanism visualization to glass. This might be a little
     * bit slow, turn it off for comp.
     */
    private static final boolean VISUALIZE = true;
    private static final double ELEVATOR_MAXIMUM_POSITION = 56.0;
    private static final double ELEVATOR_MINIMUM_POSITION = 0.0;
    private static final double ELEVATOR_REDUCTION = 2;
    /** This is wrong, it's "Sanjan units". */
    private static final double ELEVATOR_WHEEL_DIAMETER = 1;
    private static final double MAX_VEL = 190;
    private static final double MAX_ACCEL = 210;
    private static final double POSITION_TOLERANCE = 0.01;
    // private static final double VELOCITY_TOLERANCE = 0.01;

    private final OutboardLinearPositionServo m_starboardServo;
    private final OutboardLinearPositionServo m_portServo;

    private final Runnable m_viz;

    private boolean m_isSafe = false;

    public Elevator(LoggerFactory parent, int starboardID, int portID) {
        LoggerFactory child = parent.type(this);
        LoggerFactory starboardLogger = child.name("Starboard");
        LoggerFactory portLogger = child.name("Port");
        LoggerFactory starboardMotorLogger = child.name("Starboard Motor");
        LoggerFactory portMotorLogger = child.name("Port Motor");

        IncrementalProfile profile = new TrapezoidIncrementalProfile(MAX_VEL, MAX_ACCEL, POSITION_TOLERANCE);
        ProfileReference1d ref = new IncrementalProfileReference1d(profile, 0.05, 0.05);

        switch (Identity.instance) {
            case COMP_BOT -> {
                int elevatorSupplyLimit = 60;
                int elevatorStatorLimit = 90;
                PIDConstants elevatorPID = PIDConstants.makePositionPID(2.5);
                Feedforward100 elevatorFF = Feedforward100.makeKraken6Elevator();

                Kraken6Motor starboardMotor = new Kraken6Motor(starboardMotorLogger, starboardID, MotorPhase.REVERSE,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);
                Kraken6Motor portMotor = new Kraken6Motor(portMotorLogger, portID, MotorPhase.FORWARD,
                        elevatorSupplyLimit, elevatorStatorLimit, elevatorPID, elevatorFF);

                Talon6Encoder stbdEncoder = new Talon6Encoder(starboardLogger, starboardMotor);
                Talon6Encoder portEncoder = new Talon6Encoder(portLogger, portMotor);

                m_starboardServo = makeServo(starboardLogger, starboardMotor, stbdEncoder, ref);
                m_portServo = makeServo(portLogger, portMotor, portEncoder, ref);
            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(starboardMotorLogger, 600);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(portMotorLogger, 600);

                SimulatedBareEncoder stbdEncoder = new SimulatedBareEncoder(starboardLogger, starboardMotor);
                SimulatedBareEncoder portEncoder = new SimulatedBareEncoder(portLogger, portMotor);

                m_starboardServo = makeServo(starboardLogger, starboardMotor, stbdEncoder, ref);
                m_portServo = makeServo(portLogger, portMotor, portEncoder, ref);
            }

        }
        m_starboardServo.reset();
        m_portServo.reset();
        if (VISUALIZE) {
            m_viz = new ElevatorVisualization(this);
        } else {
            m_viz = () -> {
            };
        }
    }

    private static OutboardLinearPositionServo makeServo(
            LoggerFactory logger,
            BareMotor motor,
            IncrementalBareEncoder encoder,
            ProfileReference1d ref) {
        LinearMechanism mech = new LinearMechanism(
                logger,
                motor,
                encoder,
                ELEVATOR_REDUCTION,
                ELEVATOR_WHEEL_DIAMETER,
                ELEVATOR_MINIMUM_POSITION,
                ELEVATOR_MAXIMUM_POSITION);
        return new OutboardLinearPositionServo(
                logger, mech, ref, 0.5, 0.5);
    }

    public void resetElevatorProfile() {
        m_starboardServo.reset();
        m_portServo.reset();
    }

    /** Use a profile to set the position. */
    public void setPosition(double x) {
        m_starboardServo.setPositionProfiled(x, 1.3); // 54 max
        m_portServo.setPositionProfiled(x, 1.3); // 54 max
    }

    public boolean profileDone() {
        return m_starboardServo.profileDone();
    }

    public boolean atGoal() {
        return m_starboardServo.atGoal();
    }

    public void setPositionDirect(Setpoints1d x) {
        m_starboardServo.setPositionDirect(x, 1.3); // 54 max
        m_portServo.setPositionDirect(x, 1.3); // 54 max
    }

    /** set position with profile */
    public void setPositionNoGravity(double x) {
        m_starboardServo.setPositionProfiled(x, 0); // 54 max
        m_portServo.setPositionProfiled(x, 0); // 54 max
    }

    public void setStatic() {
        m_starboardServo.setPositionProfiled(m_starboardServo.getPosition().getAsDouble(), 1.3);
        // 54 max
        m_portServo.setPositionProfiled(m_portServo.getPosition().getAsDouble(), 1.3); // 54 max
    }

    public void setDutyCycle(double value) {
        m_starboardServo.setDutyCycle(value);
        m_portServo.setDutyCycle(value);
    }

    public double getPosition() {
        return m_starboardServo.getPosition().orElse(0);
    }

    public double getSetpointAcceleration() {
        // i think there are like 20 sanjan units per meter?
        // you can adjust this factor so that the wrist doesn't rotate when the elevator
        // moves.
        double sanjanAccelFactor = 20;
        return m_starboardServo.getSetpointAcceleration() / sanjanAccelFactor;
    }

    public void stop() {
        m_starboardServo.stop();
        m_portServo.stop();
    }

    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        m_isSafe = isSafe;
    }

    public void close() {
        // m_portController.close();
        // m_starboardController.close();
    }

    @Override
    public void periodic() {
        m_starboardServo.periodic();
        m_portServo.periodic();
        m_viz.run();
    }

    // OBSERVERS

    public boolean isSafeToDrive() {
        return getPosition() <= 30;
    }

    // COMMANDS

    /** Use a profile to set the position perpetually. */
    public Command set(double v) {
        return runEnd(
                () -> setPosition(v),
                () -> stop());
    }

    public Command setNoGravity(double v) {
        return runEnd(
                () -> setPositionNoGravity(v),
                () -> stop());
    }
}
