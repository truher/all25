package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.select;

import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.Config;
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
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalgamesMech extends SubsystemBase {
    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final TrajectoryPlanner m_planner;

    private final ElevatorArmWristKinematics m_kinematics;
    private final AnalyticalJacobian m_jacobian;

    private final Dynamics m_dynamics;

    private final DoubleLogger m_log_elevator;
    private final DoubleLogger m_log_shoulder;
    private final DoubleLogger m_log_wrist;

    private final DoubleLogger m_log_elevatorV;
    private final DoubleLogger m_log_shoulderV;
    private final DoubleLogger m_log_wristV;

    private final DoubleLogger m_log_elevatorA;
    private final DoubleLogger m_log_shoulderA;
    private final DoubleLogger m_log_wristA;

    private final DoubleLogger m_log_elevatorF;
    private final DoubleLogger m_log_shoulderF;
    private final DoubleLogger m_log_wristF;

    private final Pose2dLogger m_log_pose;

    private final LinearMechanism m_elevatorFront;
    private final LinearMechanism m_elevatorBack;

    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;

    public CalgamesMech(LoggerFactory log,
            double armLength,
            double wristLength) {
        LoggerFactory parent = log.type(this);
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;

        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));
        m_planner = new TrajectoryPlanner(c);

        m_kinematics = new ElevatorArmWristKinematics(armLength, wristLength);
        m_jacobian = new AnalyticalJacobian(m_kinematics);

        m_dynamics = new Dynamics();

        m_log_elevator = parent.doubleLogger(Level.TRACE, "elevatorP");
        m_log_shoulder = parent.doubleLogger(Level.TRACE, "shoulderP");
        m_log_wrist = parent.doubleLogger(Level.TRACE, "wristP");

        m_log_elevatorV = parent.doubleLogger(Level.TRACE, "elevatorV");
        m_log_shoulderV = parent.doubleLogger(Level.TRACE, "shoulderV");
        m_log_wristV = parent.doubleLogger(Level.TRACE, "wristV");

        m_log_elevatorA = parent.doubleLogger(Level.TRACE, "elevatorA");
        m_log_shoulderA = parent.doubleLogger(Level.TRACE, "shoulderA");
        m_log_wristA = parent.doubleLogger(Level.TRACE, "wristA");

        m_log_elevatorF = parent.doubleLogger(Level.TRACE, "elevatorF");
        m_log_shoulderF = parent.doubleLogger(Level.TRACE, "shoulderF");
        m_log_wristF = parent.doubleLogger(Level.TRACE, "wristF");

        m_log_pose = parent.pose2dLogger(Level.TRACE, "pose");

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
                        1.7); // TODO: calibrate upper limit 228-45

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
                        1.7); // done 9/28, raised to barge and subtracted og carriage top hight

                Kraken6Motor shoulderMotor = new Kraken6Motor(
                        shoulderLog,
                        24, // TODO: shoulder CAN ID (Done)
                        NeutralMode.BRAKE,
                        MotorPhase.REVERSE,
                        40, // og 60
                        40, // og 90
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                Talon6Encoder shoulderEncoder = new Talon6Encoder(
                        shoulderLog,
                        shoulderMotor);
                // the shoulder has a 5048 on the intermediate shaft
                AS5048RotaryPositionSensor shoulderSensor = new AS5048RotaryPositionSensor(
                        shoulderLog,
                        4, // id done
                        0.565, // TODO: verify offset
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
                        40, // og 90
                        PIDConstants.makePositionPID(5), // og 10
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
                        2); // TODO: calibrate upper limit

                SimulatedBareMotor elevatorMotorBack = new SimulatedBareMotor(elevatorbackLog, 600);
                SimulatedBareEncoder elevatorEncoderBack = new SimulatedBareEncoder(elevatorbackLog, elevatorMotorBack);
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog,
                        elevatorMotorBack,
                        elevatorEncoderBack,
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

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getHandLength() {
        return m_wristLengthM;
    }

    public Config getConfig() {
        // TODO: remove these defaults
        return new Config(
                m_elevatorBack.getPositionM().orElse(0), // only one included beacuse geared together
                m_shoulder.getPositionRad().orElse(0),
                m_wrist.getPositionRad().orElse(0));
    }

    private void logConfig(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        m_log_elevator.log(() -> c.shoulderHeight());
        m_log_shoulder.log(() -> c.shoulderAngle());
        m_log_wrist.log(() -> c.wristAngle());

        m_log_elevatorV.log(() -> jv.elevator());
        m_log_shoulderV.log(() -> jv.shoulder());
        m_log_wristV.log(() -> jv.wrist());

        m_log_elevatorA.log(() -> ja.elevator());
        m_log_shoulderA.log(() -> ja.shoulder());
        m_log_wristA.log(() -> ja.wrist());

        m_log_elevatorF.log(() -> jf.elevator());
        m_log_shoulderF.log(() -> jf.shoulder());
        m_log_wristF.log(() -> jf.wrist());

    }

    public JointVelocities getJointVelocity() {
        // TODO: think about these defaults
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
        // control has three inputs:
        // position
        Pose2d p = control.pose();
        Config c = m_kinematics.inverse(p);
        if (c.isNaN()) {
            System.out.println("skipping invalid config");
            stop();
            return;
        }
        // velocity
        JointVelocities jv = m_jacobian.inverse(control.model());
        // this is the force *of* gravity, so multiply by -1 in actuation
        // JointForce jf = m_gravity.get(c);

        JointAccelerations ja = m_jacobian.inverseA(control);

        // this is the force *required* so pass it directly.
        JointForce jf = m_dynamics.forward(c, jv, ja);

        // set each mechanism
        set(c, jv, ja, jf);
        // System.out.println(c);
        // set(c);
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
        m_elevatorFront.periodic();
        m_elevatorBack.periodic();
        m_wrist.periodic();
    }

    /////////////////////////////////
    /** This is not "hold position" this is "torque off" */
    public void stop() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    public void set(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        logConfig(c, jv, ja, jf);
        m_elevatorFront.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_elevatorBack.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_shoulder.setPosition(c.shoulderAngle(), jv.shoulder(), 0, jf.shoulder());
        m_wrist.setPosition(c.wristAngle(), jv.shoulder(), 0, jf.wrist());
    }

    /** Sets each mechanism, with zero velocity */
    public void set(Config c) {
        // this is the force *of* gravity
        // JointForce jf = m_gravity.get(c);
        // force should *oppose* gravity.

        JointVelocities jv = new JointVelocities(0, 0, 0);
        JointAccelerations ja = new JointAccelerations(0, 0, 0);
        m_jacobian.forwardA(c, jv, ja);
        JointForce jf = m_dynamics.forward(c, jv, ja);
        set(c, jv, ja, jf);
    }

    public void set(Config c, JointVelocities jv, JointAccelerations ja) {
        JointForce jf = m_dynamics.forward(c, jv, ja);
        set(c, jv, ja, jf);
    }

    private void addConfig(double height, double shoulder, double wrist) {
        Config c = getConfig();
        set(new Config(
                c.shoulderHeight() + height,
                c.shoulderAngle() + shoulder,
                c.wristAngle() + wrist));
    }

    /////////////////////////////////////////////////////////
    ///
    /// COMMANDS
    ///

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and end when done.
     */
    public Command profileHome() {
        FollowJointProfiles f = new FollowJointProfiles(this, new Config(0, 0, 0));
        return f.until(f::isDone);
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command pickWithProfile() {
        return new FollowJointProfiles(this, new Config(0, -3 * Math.PI / 4, Math.PI / 4));
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * station-pick location at rest, and stay there forever.
     */
    public Command stationWithProfile() {
        return new FollowJointProfiles(this, new Config(0, -1, 0));
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * processor location at rest, and stay there forever.
     */
    public Command processorWithProfile() {
        return new FollowJointProfiles(this, new Config(0, 1, 0));
    }

    /**
     * Use a trajectory to move from home to the L1 scoring position and hold there
     * forever
     */
    public FollowTrajectory homeToL1() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(0.5, 0.5, 1.5, 1.7))));
    }

    /**
     * Use a trajectory to move to the L2 scoring position and hold there forever
     */
    public FollowTrajectory homeToL2() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(0.85, 0.5, 2, 2))));
    }

    /**
     * Use a trajectory to move to the L3 scoring position and hold there forever
     */
    public FollowTrajectory homeToL3() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(1.2, 0.5, 2, 2))));
    }

    /**
     * Move to the L4 scoring position and hold there forever
     */
    public FollowTrajectory homeToL4() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(1.9, 0.5, 2.5, 2))));
    }

    /** ends when done */
    public Command l4ToHome() {
        FollowTrajectory f = new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1.9, 0.5, 2.5, -1),
                        HolonomicPose2d.make(1, 0, 0, Math.PI))));
        return f.until(f::isDone);
    }

    /** ends when done */
    public Command l3ToHome() {
        FollowTrajectory f = new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1.2, 0.5, 2, -1),
                        HolonomicPose2d.make(1, 0, 0, Math.PI))));
        return f.until(f::isDone);
    }

    /** ends when done */
    public Command l2ToHome() {
        FollowTrajectory f = new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(0.85, 0.5, 2, -1),
                        HolonomicPose2d.make(1, 0, 0, Math.PI))));
        return f.until(f::isDone);
    }

    /** ends when done */
    public Command l1ToHome() {
        FollowTrajectory f = new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(0.5, 0.5, 1.5, -1.0),
                        HolonomicPose2d.make(1, 0, 0, Math.PI))));
        return f.until(f::isDone);
    }

    public Command homeToAlgaeL2() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(0.85, 0.5, 1.5, 1.5))));
    }

    public Command homeToAlgaeL3() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(1.2, 0.5, 1.5, 1.5))));
    }

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        Command l3 = homeToAlgaeL3();
        Command l2 = homeToAlgaeL2();
        return select(
                Map.ofEntries(
                        Map.entry(
                                ScoringLevel.L3,
                                l3),
                        Map.entry(
                                ScoringLevel.L2,
                                l2)),
                level);
    }

    /**
     * Move to the barge scoring position and hold there forever
     */
    public FollowTrajectory homeToBarge() {
        return new FollowTrajectory(this,
                m_planner.restToRest(List.of(
                        HolonomicPose2d.make(1, 0, 0, 0),
                        HolonomicPose2d.make(2.0, 0, 1.5, 1.5))));
    }

    /** Not too far extended in any direction. */
    public boolean isSafeToDrive() {
        double x = m_elevatorBack.getPositionM().orElse(0);
        double y = m_shoulder.getPositionRad().orElse(0);
        return x < 1 && Math.abs(y) < 1;
    }

    ////////////////////

    private void addCartesian(double x, double y, double r) {
        // how much we want to change our goal point by (eg
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