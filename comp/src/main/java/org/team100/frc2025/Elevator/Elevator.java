package org.team100.frc2025.Elevator;

import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearPositionServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;

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
    private static final double maxVel = 190;
    private static final double maxAccel = 210;
    private static final double POSITION_TOLERANCE = 0.01;
    // private static final double VELOCITY_TOLERANCE = 0.01;

    private final OutboardLinearPositionServo starboardServo;
    private final OutboardLinearPositionServo portServo;

    private final Runnable m_viz;

    private boolean m_isSafe = false;

    private ScoringPosition m_targetPosition = ScoringPosition.NONE;

    public Elevator(LoggerFactory parent, int starboardID, int portID) {
        LoggerFactory child = parent.type(this);
        LoggerFactory starboardLogger = child.name("Starboard");
        LoggerFactory portLogger = child.name("Port");
        LoggerFactory starboardMotorLogger = child.name("Starboard Motor");
        LoggerFactory portMotorLogger = child.name("Port Motor");

        Profile100 profile = new TrapezoidProfile100(maxVel, maxAccel, POSITION_TOLERANCE);
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

                LinearMechanism starboardMech = new LinearMechanism(
                        starboardMotor,
                        stbdEncoder,
                        ELEVATOR_REDUCTION,
                        ELEVATOR_WHEEL_DIAMETER,
                        ELEVATOR_MINIMUM_POSITION,
                        ELEVATOR_MAXIMUM_POSITION);

                LinearMechanism portMech = new LinearMechanism(
                        portMotor,
                        portEncoder,
                        ELEVATOR_REDUCTION,
                        ELEVATOR_WHEEL_DIAMETER,
                        ELEVATOR_MINIMUM_POSITION,
                        ELEVATOR_MAXIMUM_POSITION);

                starboardServo = new OutboardLinearPositionServo(
                        starboardLogger,
                        starboardMech,
                        ref);
                portServo = new OutboardLinearPositionServo(
                        portLogger,
                        portMech,
                        ref);

            }
            default -> {
                SimulatedBareMotor starboardMotor = new SimulatedBareMotor(starboardMotorLogger, 600);
                SimulatedBareMotor portMotor = new SimulatedBareMotor(portMotorLogger, 600);

                SimulatedBareEncoder stbdEncoder = new SimulatedBareEncoder(starboardLogger, starboardMotor);
                SimulatedBareEncoder portEncoder = new SimulatedBareEncoder(portLogger, portMotor);

                LinearMechanism starboardMech = new LinearMechanism(
                        starboardMotor,
                        stbdEncoder,
                        ELEVATOR_REDUCTION,
                        ELEVATOR_WHEEL_DIAMETER,
                        ELEVATOR_MINIMUM_POSITION,
                        ELEVATOR_MAXIMUM_POSITION);
                LinearMechanism portMech = new LinearMechanism(
                        portMotor,
                        portEncoder,
                        ELEVATOR_REDUCTION,
                        ELEVATOR_WHEEL_DIAMETER,
                        ELEVATOR_MINIMUM_POSITION,
                        ELEVATOR_MAXIMUM_POSITION);

                starboardServo = new OutboardLinearPositionServo(
                        starboardLogger,
                        starboardMech,
                        ref);
                portServo = new OutboardLinearPositionServo(
                        portLogger,
                        portMech,
                        ref);
            }

        }
        starboardServo.reset();
        portServo.reset();
        if (VISUALIZE) {
            m_viz = new ElevatorVisualization(this);
        } else {
            m_viz = () -> {
            };
        }
    }

    public void resetElevatorProfile() {
        starboardServo.reset();
        portServo.reset();
    }

    public void setPosition(double x) {
        starboardServo.setPositionProfiled(x, 1.3); // 54 max
        portServo.setPositionProfiled(x, 1.3); // 54 max
    }

    public boolean profileDone() {
        return starboardServo.profileDone();
    }

    public void setPositionDirect(Setpoints1d x) {
        starboardServo.setPositionDirect(x, 1.3); // 54 max
        portServo.setPositionDirect(x, 1.3); // 54 max
    }

    public void setPositionNoGravity(double x) {
        starboardServo.setPositionProfiled(x, 0); // 54 max
        portServo.setPositionProfiled(x, 0); // 54 max
    }

    public void setStatic() {
        starboardServo.setPositionProfiled(starboardServo.getPosition().getAsDouble(), 1.3);
        // 54 max
        portServo.setPositionProfiled(portServo.getPosition().getAsDouble(), 1.3); // 54 max
    }

    public void setDutyCycle(double value) {
        starboardServo.setDutyCycle(value);
        portServo.setDutyCycle(value);
    }

    public double getPosition() {
        return starboardServo.getPosition().orElse(0);
    }

    public double getSetpointAcceleration() {
        // i think there arel like 20 sanjan units per meter?
        // you can adjust this factor so that the wrist doesn't rotate when the elevator
        // moves.
        double sanjanAccelFactor = 20;
        return starboardServo.getSetpointAcceleration() / sanjanAccelFactor;
    }

    public void stop() {
        starboardServo.stop();
        portServo.stop();
    }

    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        m_isSafe = isSafe;
    }

    public void setTargetScoringPosition(ScoringPosition position) {
        m_targetPosition = position;
    }

    public ScoringPosition getScoringPosition() {
        return m_targetPosition;
    }

    public void close() {
        // m_portController.close();
        // m_starboardController.close();
    }

    @Override
    public void periodic() {
        starboardServo.periodic();
        portServo.periodic();
        m_viz.run();
    }
}
