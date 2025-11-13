package org.team100.frc2025;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.subsystems.mecanum.MecanumDrive100;
import org.team100.lib.subsystems.r3.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.r3.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.r3.commands.VelocityFeedforwardOnly;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.DiamondConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Autons {
    private static final Pose2d KNIGHT_MOVE = new Pose2d(2, 1, Rotation2d.kZero);
    private static final Pose2d ONE = new Pose2d(1, 0, Rotation2d.kZero); // for calibration, 1 meter forward (1.51
                                                                          // works)
    private static final Pose2d TWO = new Pose2d(1, 4, Rotation2d.k180deg);
    private static final Pose2d FOUR = new Pose2d(4, 4, Rotation2d.kCCW_90deg);

    private final AutonChooser m_autonChooser;
    private final MecanumDrive100 m_drive;
    private final HolonomicProfile m_profile;
    private final TrajectoryPlanner m_planner;
    private final TrajectoryVisualization m_viz;
    private final DualDrumShooter m_shooter;
    private final IndexerServo m_indexer;

    public Autons(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            MecanumDrive100 drive,
            IndexerServo indexer,
            DualDrumShooter shooter) {
        LoggerFactory log = parent.name("Auton");
        m_autonChooser = new AutonChooser();
        m_drive = drive;
        m_indexer = indexer;
        m_shooter = shooter;
        m_profile = HolonomicProfile.wpi(4, 8, 3, 6);
        List<TimingConstraint> constraints = List.of(
                new DiamondConstraint(log, 2, 2, 2),
                new ConstantConstraint(log, 1, 1),
                new YawRateConstraint(log, 1, 1));
        m_planner = new TrajectoryPlanner(constraints);
        m_viz = new TrajectoryVisualization(fieldLogger);

        ControllerR3 controller = ControllerFactoryR3.byIdentity(log);

        MoveAndHold knight_move = new VelocityFeedforwardOnly(log, m_profile, KNIGHT_MOVE, m_drive);
        m_autonChooser.add("knight_move",
                new AnnotatedCommand(knight_move.until(knight_move::isDone).withName("auto knight_move"), null, null));

        MoveAndHold one = new VelocityFeedforwardOnly(log, m_profile, ONE, m_drive);
        m_autonChooser.add("one",
                new AnnotatedCommand(one.until(one::isDone).withName("auto one"), null, null));

        MoveAndHold two = new VelocityFeedforwardOnly(log, m_profile, TWO, m_drive);
        m_autonChooser.add("two",
                new AnnotatedCommand(two.until(two::isDone).withName("auto two"), null, null));

        Command three = m_drive.driveWithGlobalVelocity(
                new GlobalVelocityR3(1.5, 0, 0)).withTimeout(1.0);
        m_autonChooser.add("three",
                new AnnotatedCommand(three.withName("auto three"), null, null));

        MoveAndHold four = new DriveToPoseWithProfile(
                log, drive, controller, m_profile, () -> FOUR);
        m_autonChooser.add("four",
                new AnnotatedCommand(four.until(four::isDone).withName("auto four"), null, null));

        MoveAndHold standard = new DriveWithTrajectoryFunction(
                log, drive, controller, m_viz, this::standard);
        m_autonChooser.add("standard",
                new AnnotatedCommand(
                        standard.until(standard::isDone)
                                .andThen(new Shoot(m_shooter, m_indexer, 9))
                                .withTimeout(2)
                                .withName("auto standard"),
                        null, null));

        MoveAndHold standard2 = new DriveWithTrajectoryFunction(
                log, drive, controller, m_viz, this::standard2);
        m_autonChooser.add("standard2",
                new AnnotatedCommand(
                        standard2.until(standard2::isDone)
                                .andThen(
                                        new Shoot(m_shooter, m_indexer, 8).withTimeout(2))
                                .withName("auto standard"),
                        null, new Pose2d(.48, 0.53, Rotation2d.kZero)));
        //

        MoveAndHold calib = new DriveWithTrajectoryFunction(
                log, drive, controller, m_viz, this::calib);
        m_autonChooser.add("calibration",
                new AnnotatedCommand(calib.until(calib::isDone).withName("auto calib"), null, null));

        /*
         * MoveAndHold autolow = new DriveWithTrajectoryFunction(
         * log, drive, controller, m_viz, this::autolow);
         * m_autonChooser.add("straight low auto",
         * new AnnotatedCommand(
         * autolow.until(autolow::isDone).andThen(pivot.extend().withTimeout(1)).
         * withName("auto autolow"),
         * null, null));
         */
        // new AnnotatedCommand(pivot.extend().withTimeout(1).withName("auto autolow"),
        // null, null));

    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }

    private Trajectory100 standard(Pose2d p) {
        Pose2d end = new Pose2d(p.getX() + 1, p.getY() + 0, p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, Math.toRadians(0))));
    }

    private Trajectory100 standard2(Pose2d p) {
        Pose2d end = new Pose2d(p.getX() + 1.4, p.getY() + 0, p.getRotation().plus(Rotation2d.fromDegrees(  5)));
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, Math.toRadians(0))));
    }

    private Trajectory100 calib(Pose2d p) {
        Pose2d end = new Pose2d(p.getX(), p.getY() + 1, p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, Math.PI / 2)));
    }
    /*
     * private Trajectory100 swerve(Pose2d p) {
     * Pose2d end = new Pose2d(p.getX(), p.getY() - 1, p.getRotation());
     * return m_planner.restToRest(List.of(
     * HolonomicPose2d.make(p, 0),
     * HolonomicPose2d.make(end, Math.PI / 2)));
     * }
     */

}
