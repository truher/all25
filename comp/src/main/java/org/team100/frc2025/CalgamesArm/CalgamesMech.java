package org.team100.frc2025.CalgamesArm;

import java.util.function.DoubleSupplier;

import org.team100.lib.commands.r3.SubsystemR3;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.GearedRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.ElevatorArmWristKinematics;
import org.team100.lib.motion.Mech;
import org.team100.lib.motion.MechInterface;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalgamesMech extends SubsystemBase implements MechInterface, SubsystemR3 {
    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final ElevatorArmWristKinematics m_kinematics;
    private final Jacobian m_jacobian;
    private Config m_config;

    private final Gravity m_gravity;

    private final DoubleLogger m_log_elevator;
    private final DoubleLogger m_log_shoulder;
    private final DoubleLogger m_log_wrist;

    private final Pose2dLogger m_log_pose;

    private final LinearMechanism m_elevator;
    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;

    private static final double ARM_REDUCTION = 78; // number from om, from talon to output? (inspo from elevator code)

    public CalgamesMech(LoggerFactory log,
            double armLength,
            double wristLength) {
        LoggerFactory parent = log.type(this);
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;
        m_kinematics = new ElevatorArmWristKinematics(armLength, wristLength);
        m_jacobian = new Jacobian(m_kinematics);

        m_gravity = Gravity.from2025();

        m_log_elevator = parent.doubleLogger(Level.TRACE, "elevator");
        m_log_shoulder = parent.doubleLogger(Level.TRACE, "shoulder");
        m_log_wrist = parent.doubleLogger(Level.TRACE, "wrist");
        m_log_pose = parent.pose2dLogger(Level.TRACE, "pose");

        LoggerFactory elevatorLog = parent.name("elevator");
        LoggerFactory shoulderLog = parent.name("shoulder");
        LoggerFactory wristLog = parent.name("wrist");
        switch (Identity.instance) {
            case COMP_BOT -> {
                Kraken6Motor elevatorMotor = new Kraken6Motor(
                        elevatorLog,
                        11, // TODO: elevator CAN ID (DID, now for starboard)
                        MotorPhase.FORWARD,
                        60,
                        90,
                        PIDConstants.makePositionPID(10),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder elevatorEncoder = new Talon6Encoder(
                        elevatorLog, elevatorMotor);
                m_elevator = new LinearMechanism(
                        elevatorLog,
                        elevatorMotor,
                        elevatorEncoder,
                        2, // TODO: calibrate ratio
                        0.05, // TODO: calibrate pulley size
                        0, // TODO: calibrate lower limit
                        2); // TODO: calibrate upper limit

                Kraken6Motor shoulderMotor = new Kraken6Motor(
                        parent,
                        24, // TODO: shoulder CAN ID (Done)
                        MotorPhase.FORWARD,
                        60,
                        90,
                        PIDConstants.makePositionPID(10),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder shoulderEncoder = new Talon6Encoder(
                        shoulderLog,
                        shoulderMotor);
                // the shoulder has a 5048 on the intermediate shaft
                AS5048RotaryPositionSensor shoulderSensor = new AS5048RotaryPositionSensor(
                        shoulderLog,
                        4, // id done
                        0.059573, // TODO: verify offset
                        EncoderDrive.DIRECT); // TODO: verify drive
                GearedRotaryPositionSensor gearedSensor = new GearedRotaryPositionSensor(
                        shoulderSensor,
                        10); // TODO: verify gear ratio

                ProxyRotaryPositionSensor shoulderProxySensor = new ProxyRotaryPositionSensor(
                        shoulderEncoder, // what is this - kym
                        100); // TODO: calibrate ratio
                CombinedRotaryPositionSensor shoulderCombined = new CombinedRotaryPositionSensor(
                        shoulderLog,
                        gearedSensor,
                        shoulderProxySensor);
                m_shoulder = new RotaryMechanism(
                        shoulderLog,
                        shoulderMotor, // need to learn what these three things do and how rotatrymech works - kym
                        shoulderCombined,
                        100, // TODO: calibrate ratio
                        -3, // TODO: calibrate lower limit
                        3);// TODO: calibrate upper limit

                Kraken6Motor wristMotor = new Kraken6Motor(
                        parent,
                        22, // TODO: wrist CAN ID (Done)
                        MotorPhase.FORWARD,
                        60,
                        90,
                        PIDConstants.makePositionPID(10),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                // the wrist has no angle sensor, so it needs to start in the "zero" position,
                // or we need to add a homing
                Talon6Encoder wristEncoder = new Talon6Encoder(
                        wristLog, wristMotor);
                ProxyRotaryPositionSensor wristProxySensor = new ProxyRotaryPositionSensor(
                        wristEncoder,
                        2* (38/12)*(38/12)*(50/12));
                wristProxySensor.setEncoderPosition(2); // 9/27/25 measured
                m_wrist = new RotaryMechanism(
                        wristLog,
                        wristMotor,
                        wristProxySensor,
                        58, // TODO: calibrate ratio
                        -3, // TODO: calibrate lower limit
                        3);// TODO: calibrate upper limit

            }
            default -> {
                SimulatedBareMotor elevatorMotor = new SimulatedBareMotor(elevatorLog, 600);
                SimulatedBareEncoder elevatorEncoder = new SimulatedBareEncoder(elevatorLog, elevatorMotor);
                m_elevator = new LinearMechanism(
                        elevatorLog,
                        elevatorMotor,
                        elevatorEncoder,
                        2, // TODO: calibrate ratio
                        0.05, // TODO: calibrate pulley size
                        0, // TODO: calibrate lower limit
                        2); // TODO: calibrate upper limit

                SimulatedBareMotor shoulderMotor = new SimulatedBareMotor(shoulderLog, 600);
                SimulatedBareEncoder shoulderEncoder = new SimulatedBareEncoder(shoulderLog, shoulderMotor);
                SimulatedRotaryPositionSensor shoulderSensor = new SimulatedRotaryPositionSensor(
                        shoulderLog,
                        shoulderEncoder,
                        100); // TODO: calibrate gear ratio
                m_shoulder = new RotaryMechanism(
                        shoulderLog,
                        shoulderMotor,
                        shoulderSensor,
                        100, // TODO: calibrate gear ratio
                        -3, // TODO: calibrate lower limit
                        3); // TODO: calibrate upper limit

                SimulatedBareMotor wristMotor = new SimulatedBareMotor(wristLog, 600);
                SimulatedBareEncoder wristEncoder = new SimulatedBareEncoder(wristLog, wristMotor);
                SimulatedRotaryPositionSensor wristSensor = new SimulatedRotaryPositionSensor(
                        wristLog,
                        wristEncoder,
                        58); // TODO: calibrate gear ratio
                m_wrist = new RotaryMechanism(
                        wristLog,
                        wristMotor,
                        wristSensor,
                        58, // TODO: calibrate gear ratio
                        -3, // TODO: calibrate lower limit
                        3);
            }
        }// TODO: calibrate upper limit
    }

    // simple methods grabbed from mech.java
    public static Mech make2025(LoggerFactory parent) {
        return new Mech(0.3, 0.1); // these values are prolly wrong, needs to be measured IRL
    }

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getHandLength() {
        return m_wristLengthM;
    }

    public Config getConfig() {
        // TODO: remove these defaults
        return new Config(
                m_elevator.getPositionM().orElse(0),
                m_shoulder.getPositionRad().orElse(0),
                m_wrist.getPositionRad().orElse(0));
    }

    private void logConfig(Config c) {
        m_log_elevator.log(() -> c.shoulderHeight());
        m_log_shoulder.log(() -> c.shoulderAngle());
        m_log_wrist.log(() -> c.wristAngle());
    }

    private JointVelocities getJointVelocity() {
        // TODO: think about these defaults
        return new JointVelocities(
                m_elevator.getVelocityM_S().orElse(0),
                m_shoulder.getVelocityRad_S().orElse(0),
                m_wrist.getVelocityRad_S().orElse(0));
    }

    public SwerveModel getState() {
        Config c = getConfig();
        JointVelocities jv = getJointVelocity();
        Pose2d p = m_kinematics.forward(c);
        FieldRelativeVelocity v = m_jacobian.forward(c, jv);
        return new SwerveModel(p, v);
    }

    /** There are no profiles here, so this control needs to be feasible. */
    public void set(SwerveControl control) {
        // control has three inputs:
        // position
        Pose2d p = control.pose();
        Config c = m_kinematics.inverse(p);
        logConfig(c);
        if (c.isNaN()) {
            System.out.println("skipping invalid config");
            stop();
            return;
        }
        // velocity
        JointVelocities jv = m_jacobian.inverse(control.model());
        // this is the force *of* gravity
        JointForce jf = m_gravity.get(c);
        // set each mechanism
        // force should *oppose* gravity.
        m_elevator.setPosition(c.shoulderHeight(), jv.elevator(), 0, -1.0 * jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), jv.shoulder(), 0, -1.0 * jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), jv.wrist(), 0, -1.0 * jf.wrist());
    }

    private void setConfig(Config config) {
        m_config = config;
    }

    public Command config(
            DoubleSupplier height,
            DoubleSupplier shoulder,
            DoubleSupplier wrist) {
        return run(() -> addConfig(
                height.getAsDouble(), shoulder.getAsDouble(), wrist.getAsDouble()));
    }

    /** Controls are cartesian axes. */
    public Command cartesian(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier r) {
        return run(() -> addCartesian(x.getAsDouble(), y.getAsDouble(), r.getAsDouble()));
    }

    @Override
    public void periodic() {
        m_shoulder.periodic();
        m_elevator.periodic();
        m_wrist.periodic();
    }

    /////////////////////////////////
    /** This is not "hold position" this is "torque off" */
    public void stop() {
        m_elevator.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    /** Sets each mechanism, with zero velocity */
    public void set(Config c) {
        logConfig(c);
        // this is the force *of* gravity
        JointForce jf = m_gravity.get(c);
        // force should *oppose* gravity.
        m_elevator.setPosition(c.shoulderHeight(), 0, 0, -1.0 * jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), 0, 0, -1.0 * jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), 0, 0, -1.0 * jf.wrist());
    }

    private void addConfig(double height, double shoulder, double wrist) {
        Config c = getConfig();
        set(new Config(
                c.shoulderHeight() + height,
                c.shoulderAngle() + shoulder,
                c.wristAngle() + wrist));
    }

    private void addCartesian(double x, double y, double r) { // how much we want to change our goal point by (eg
        // move up by 5, left by 3, rotate 20 degrees)
        Pose2d p = m_kinematics.forward(getConfig());
        // Pose2d.add() method uses the pose frame so we don't use it.
        // We transform p in the global frame by adding components
        double x2 = p.getX() + x;
        double y2 = p.getY() + y;
        Rotation2d r2 = p.getRotation().plus(new Rotation2d(r));
        Pose2d newP = new Pose2d(x2, y2, r2); // our new goal point
        m_log_pose.log(() -> newP); // solves for the config to reach new goal point
        Config c = m_kinematics.inverse(newP);
        if (c.isNaN()) {
            System.out.println("skipping invalid config");
            stop(); // if the config fails, we quit
            return;
        }
        set(c);
    }

    // all of the above is implementing the math, not actually moving the motors or
    // mechansim
    // so need to add the like set position methods, like those in eleveator (use
    // desmos kai)

    /*
     * notes:
     * calgamesarm class should be dumb, basically just figuring out how much to
     * move things and moving them
     * 
     * should make a seperate class for the needed trajecotries and figuring them
     * out
     * as part of that, we should make a thing to graph the changes in wrist,
     * shoulder, elevator position (derive the trajectory)
     * and make sure there are no discontiuites, those mean singulairty (i think)
     * 
     * will make a seperate class that translates the points along the trajectory
     * into simple poses accepatable by the dumb class
     * 
     */

}