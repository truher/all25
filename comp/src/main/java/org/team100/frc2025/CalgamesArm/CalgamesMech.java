package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.select;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.lib.commands.Done;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
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
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ConfigLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeAccelerationLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.JointAccelerationsLogger;
import org.team100.lib.logging.LoggerFactory.JointForceLogger;
import org.team100.lib.logging.LoggerFactory.JointVelocitiesLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointForce;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalgamesMech extends SubsystemBase {
    private static final boolean DEBUG = false;

    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final MechTrajectories m_transit;

    private final ElevatorArmWristKinematics m_kinematics;
    private final AnalyticalJacobian m_jacobian;

    private final Dynamics m_dynamics;

    private final ConfigLogger m_log_config;
    private final JointVelocitiesLogger m_log_jointV;
    private final JointAccelerationsLogger m_log_jointA;
    private final JointForceLogger m_log_jointF;

    private final Pose2dLogger m_log_pose;
    private final FieldRelativeVelocityLogger m_log_cartesianV;
    private final FieldRelativeAccelerationLogger m_log_cartesianA;

    private final LinearMechanism m_elevatorFront;
    private final LinearMechanism m_elevatorBack;

    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;
    double m_homeHeight = 0.843; // sum of the lengths of the arms

    public CalgamesMech(LoggerFactory log,
            double armLength,
            double wristLength) {
        LoggerFactory parent = log.type(this);
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;

        m_kinematics = new ElevatorArmWristKinematics(armLength, wristLength);
        m_jacobian = new AnalyticalJacobian(m_kinematics);
        m_dynamics = new Dynamics();

        m_transit = new MechTrajectories(this, m_kinematics, m_jacobian);

        LoggerFactory jointLog = parent.name("joints");
        m_log_config = jointLog.logConfig(Level.TRACE, "config");
        m_log_jointV = jointLog.logJointVelocities(Level.TRACE, "velocity");
        m_log_jointA = jointLog.logJointAccelerations(Level.TRACE, "accel");
        m_log_jointF = jointLog.logJointForce(Level.TRACE, "force");

        LoggerFactory cartesianLog = parent.name("cartesian");
        m_log_pose = cartesianLog.pose2dLogger(Level.TRACE, "pose");
        m_log_cartesianV = cartesianLog.fieldRelativeVelocityLogger(Level.TRACE, "velocity");
        m_log_cartesianA = cartesianLog.fieldRelativeAccelerationLogger(Level.TRACE, "accel");

        LoggerFactory elevatorbackLog = parent.name("elevatorBack");
        LoggerFactory elevatorfrontLog = parent.name("elevatorFront");
        LoggerFactory shoulderLog = parent.name("shoulder");
        LoggerFactory wristLog = parent.name("wrist");
        switch (Identity.instance) {
            case COMP_BOT -> {
                Kraken6Motor elevatorFrontMotor = new Kraken6Motor(
                        elevatorfrontLog,
                        11, // TODO: elevator CAN ID (DID, now for starboard)
                        NeutralMode.BRAKE,
                        MotorPhase.REVERSE,
                        100,
                        100,
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder elevatorFrontEncoder = new Talon6Encoder(
                        elevatorfrontLog, elevatorFrontMotor);
                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog,
                        elevatorFrontMotor,
                        elevatorFrontEncoder,
                        2.182, // TODO: calibrate ratio
                        0.03844, // TODO: calibrate pulley size- done
                        0, // TODO: calibrate lower limit
                        1.9); // TODO: calibrate upper limit 228-45

                Kraken6Motor elevatorBackMotor = new Kraken6Motor(
                        elevatorbackLog,
                        12, // TODO: elevator CAN ID (DID, now for port)
                        NeutralMode.BRAKE,
                        MotorPhase.FORWARD,
                        100, // orginally 60
                        100, // originally 90
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder elevatorBackEncoder = new Talon6Encoder(
                        elevatorbackLog, elevatorBackMotor);
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog,
                        elevatorBackMotor,
                        elevatorBackEncoder,
                        2.182, // done 9/28
                        0.03844, // done 9/28
                        0, // just 0
                        1.9); // done 9/28, raised to barge and subtracted og carriage top hight

                Kraken6Motor shoulderMotor = new Kraken6Motor(
                        shoulderLog,
                        24, // TODO: shoulder CAN ID (Done)
                        NeutralMode.BRAKE,
                        MotorPhase.REVERSE,
                        100, // og 60
                        100, // og 90
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder shoulderEncoder = new Talon6Encoder(
                        shoulderLog,
                        shoulderMotor);
                // the shoulder has a 5048 on the intermediate shaft
                AS5048RotaryPositionSensor shoulderSensor = new AS5048RotaryPositionSensor(
                        shoulderLog,
                        4, // id done
                        0.684, // TODO: verify offset
                        EncoderDrive.INVERSE); // verified drive - 9/28
                GearedRotaryPositionSensor gearedSensor = new GearedRotaryPositionSensor(
                        shoulderSensor,
                        8); // verified gear ratio - 9/28/25

                ProxyRotaryPositionSensor shoulderProxySensor = new ProxyRotaryPositionSensor(
                        shoulderEncoder, // what is this - kym
                        78); // TODO: calibrate ratio - 9/28/25
                CombinedRotaryPositionSensor shoulderCombined = new CombinedRotaryPositionSensor(
                        shoulderLog,
                        gearedSensor,
                        shoulderProxySensor);
                m_shoulder = new RotaryMechanism(
                        shoulderLog,
                        shoulderMotor, // need to learn what these three things do and how rotatrymech works - kym
                        shoulderCombined,
                        78, // TODO: calibrate ratio - 9/28
                        -2, // TODO: calibrate lower limit (DO IT FOR REAL)
                        2);// TODO: calibrate upper limit (DO IT FOR REAL)

                Kraken6Motor wristMotor = new Kraken6Motor(
                        wristLog,
                        22, // TODO: wrist CAN ID (Done)
                        NeutralMode.COAST,
                        MotorPhase.FORWARD,
                        40, // og 60
                        60, // og 90
                        PIDConstants.makePositionPID(8), // og 10
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                // the wrist has no angle sensor, so it needs to start in the "zero" position,
                // or we need to add a homing
                Talon6Encoder wristEncoder = new Talon6Encoder(
                        wristLog, wristMotor);
                double gearRatio = 55.710;
                ProxyRotaryPositionSensor wristProxySensor = new ProxyRotaryPositionSensor(
                        wristEncoder,
                        gearRatio);
                double wristEncoderOffset = 2.06818; // 2+0.06818
                wristProxySensor.setEncoderPosition(wristEncoderOffset); // 9/27/25 measured

                m_wrist = new RotaryMechanism(
                        wristLog,
                        wristMotor,
                        wristProxySensor,
                        gearRatio, // - 9/28
                        -1.5, // - 9/28
                        2.1);// -9/28, decided around starting position

            }
            default -> {
                SimulatedBareMotor elevatorMotorFront = new SimulatedBareMotor(elevatorfrontLog, 600);
                SimulatedBareEncoder elevatorEncoderFront = new SimulatedBareEncoder(elevatorfrontLog,
                        elevatorMotorFront);
                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog,
                        elevatorMotorFront,
                        elevatorEncoderFront,
                        2, // TODO: calibrate ratio
                        0.05, // TODO: calibrate pulley size
                        0, // TODO: calibrate lower limit
                        2.2); // TODO: calibrate upper limit

                SimulatedBareMotor elevatorMotorBack = new SimulatedBareMotor(elevatorbackLog, 600);
                SimulatedBareEncoder elevatorEncoderBack = new SimulatedBareEncoder(elevatorbackLog, elevatorMotorBack);
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog,
                        elevatorMotorBack,
                        elevatorEncoderBack,
                        2, // TODO: calibrate ratio
                        0.05, // TODO: calibrate pulley size
                        0, // TODO: calibrate lower limit
                        2.2); // TODO: calibrate upper limit

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
        }
    }

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getHandLength() {
        return m_wristLengthM;
    }

    public Config getConfig() {
        return new Config(
                m_elevatorBack.getPositionM().orElse(0),
                m_shoulder.getPositionRad().orElse(0),
                m_wrist.getPositionRad().orElse(0));
    }

    public JointVelocities getJointVelocity() {
        return new JointVelocities(
                m_elevatorBack.getVelocityM_S().orElse(0),
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
        Pose2d pose = control.pose();
        Config config = m_kinematics.inverse(pose);
        if (DEBUG)
            System.out.printf("pose %s config %s\n", Util.pose2Str(pose), config);
        if (config.isNaN()) {
            if (DEBUG)
                System.out.println("skipping invalid config");
            stop();
            return;
        }
        JointVelocities jv = m_jacobian.inverse(control.model());
        JointAccelerations ja = m_jacobian.inverseA(control);
        set(config, jv, ja);
    }

    public void set(Config c, JointVelocities jv, JointAccelerations ja) {
        JointForce jf = m_dynamics.forward(c, jv, ja);
        set(c, jv, ja, jf);
    }

    /** This is not "hold position" this is "torque off" */
    public void stop() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    /////////////////////////////////////////////////////////
    ///
    /// COMMANDS
    ///

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and end when done.
     */
    public Command profileHomeTerminal() {
        // FollowJointProfiles f =
        // FollowJointProfiles.WithCurrentLimitedExponentialProfile(
        // this, new Config(0, 0, 0));
        FollowJointProfiles f = FollowJointProfiles.slowFast(
                this, new Config(0, 0, 0));
        return f.until(f::isDone)
                .withName("profileHomeTerminal");
    }

    public Command profileHomeEndless() {
        // return FollowJointProfiles.WithCurrentLimitedExponentialProfile(
        // this, new Config(0, 0, 0));
        return FollowJointProfiles.slowFast(
                this, new Config(0, 0, 0))
                .withName("profileHomeEndless");
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command pickWithProfile() {
        // return FollowJointProfiles.WithCurrentLimitedExponentialProfile(
        // this, new Config(0, -1.83, -0.12));
        return FollowJointProfiles.fastSlow(
                this, new Config(0, -1.83, -0.12))
                .withName("pickWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * station-pick location at rest, and stay there forever.
     */
    public Command stationWithProfile() {
        // return FollowJointProfiles.WithCurrentLimitedExponentialProfile(
        // this, new Config(0, -1, 0));
        return FollowJointProfiles.fastSlow(
                this, new Config(0, -1, 0))
                .withName("stationWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * processor location at rest, and stay there forever.
     */
    public Command processorWithProfile() {
        // return FollowJointProfiles.WithCurrentLimitedExponentialProfile(
        // this, new Config(0, 1, 0));
        return FollowJointProfiles.fastSlow(
                this, new Config(0, 1, 0))
                .withName("processorWithProfile");
    }

    public Done homeToL1() {
        return m_transit.endless("homeToL1",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 0),
                HolonomicPose2d.make(0.5, 0.5, 1.5, 1.7));
    }

    public Done homeToL2() {
        return m_transit.endless("homeToL2",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 1.5),
                HolonomicPose2d.make(0.52, 0.54, 2, 1.5));
    }

    public Done homeToL3() {
        return m_transit.endless("homeToL3",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 0.8),
                HolonomicPose2d.make(0.92, 0.56, 1.7, 1.5));
    }

    public Done homeToL4() {
        return m_transit.endless("homeToL4",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 0.1),
                HolonomicPose2d.make(1.57, 0.54, 2, 1.57));
    }

    public Command l4ToHome() {
        return m_transit.terminal("l4ToHome",
                HolonomicPose2d.make(1.57, 0.54, 2.5, -1),
                HolonomicPose2d.make(m_homeHeight, 0, 0, Math.PI));
    }

    public Command l3ToHome() {
        return m_transit.terminal("l3ToHome",
                HolonomicPose2d.make(0.92, 0.56, 1.7, -1.5),
                HolonomicPose2d.make(m_homeHeight, 0, 0,-2.3 ));
    }

    public Command l2ToHome() {
        return m_transit.terminal("l2ToHome",
                HolonomicPose2d.make(0.52, 0.54``, 2, -1.5),
                HolonomicPose2d.make(m_homeHeight, 0, 0, -1.5));
    }

    public Command l1ToHome() {
        return m_transit.terminal("l1ToHome",
                HolonomicPose2d.make(0.5, 0.5, 1.5, -1.0),
                HolonomicPose2d.make(m_homeHeight, 0, 0, Math.PI));
    }

    public Command homeToAlgaeL2() {
        return m_transit.endless("homeToAlgaeL2",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 0),
                HolonomicPose2d.make(0.85, 0.7, 1.5, 1.5));
    }

    public Command homeToAlgaeL3() {
        return m_transit.endless("homeToAlgaeL3",
                HolonomicPose2d.make(m_homeHeight, 0, 0, 0),
                HolonomicPose2d.make(1.2, 0.7, 1.5, 1.5));
    }

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        return select(Map.ofEntries(
                Map.entry(ScoringLevel.L3, homeToAlgaeL3()),
                Map.entry(ScoringLevel.L2, homeToAlgaeL2()) //
        ), level)
                .withName("algaeReefPick");
    }

    /**
     * Move to the barge scoring position and hold there forever
     */
    public Done homeToBarge() {
        return m_transit.endless("homeToBarge",
                HolonomicPose2d.make(1, 0, 0, 0),
                HolonomicPose2d.make(1.7, -0.5, -1.5, 1.5));
    }
    public Done bargeToHome() {
        return m_transit.endless("bargeToHome",
                HolonomicPose2d.make(1.7, -0.5, -1.5, -1.5),     
                HolonomicPose2d.make(1, 0, 0, 0));
                
    }

    /** Not too far extended in any direction. */
    public boolean isSafeToDrive() {
        double x = m_elevatorBack.getPositionM().orElse(0);
        double y = m_shoulder.getPositionRad().orElse(0);
        return x < 1 && Math.abs(y) < 1;
    }

    /////////////////////////////////////////////////////////////

    public ElevatorArmWristKinematics getKinematics() {
        return m_kinematics;
    }

    public AnalyticalJacobian getJacobian() {
        return m_jacobian;
    }

    @Override
    public void periodic() {
        m_shoulder.periodic();
        m_elevatorFront.periodic();
        m_elevatorBack.periodic();
        m_wrist.periodic();
    }

    /////////////////////////////////////////////////////////////

    private void set(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        logConfig(c, jv, ja, jf);
        m_elevatorFront.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_elevatorBack.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), jv.shoulder(), 0, jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), jv.shoulder(), 0, jf.wrist());
    }

    private void logConfig(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        m_log_config.log(() -> c);
        m_log_jointV.log(() -> jv);
        m_log_jointA.log(() -> ja);
        m_log_jointF.log(() -> jf);
        Pose2d p = m_kinematics.forward(c);
        FieldRelativeVelocity v = m_jacobian.forward(c, jv);
        FieldRelativeAcceleration a = m_jacobian.forwardA(c, jv, ja);
        m_log_pose.log(() -> p);
        m_log_cartesianV.log(() -> v);
        m_log_cartesianA.log(() -> a);
    }

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