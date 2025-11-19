package org.team100.frc2025;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.field.MechanicalMayhem2025;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.subsystems.mecanum.MecanumDrive100;
import org.team100.lib.subsystems.r3.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.r3.commands.VelocityFeedforwardOnly;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.DiamondConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class Autons {
    private static final Pose2d KNIGHT_MOVE = new Pose2d(2, 1, Rotation2d.kZero);
    private static final Pose2d ONE = new Pose2d(1, 0, Rotation2d.kZero); // for calibration, 1 meter forward (1.51
                                                                          // works)
    private static final Pose2d TWO = new Pose2d(1, 4, Rotation2d.k180deg);
    private static final Pose2d FOUR = new Pose2d(4, 4, Rotation2d.kCCW_90deg);

    private final LoggerFactory m_log;
    private final AutonChooser m_autonChooser;
    private final MecanumDrive100 m_drive;
    private final HolonomicProfile m_profile;
    private final TrajectoryPlanner m_planner;
    private final TrajectoryVisualization m_viz;
    private final ControllerR3 m_controller;

    public Autons(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            MecanumDrive100 drive,
            Pivot pivot) {
        m_log = parent.name("Auton");
        m_autonChooser = new AutonChooser();
        m_drive = drive;
        m_profile = HolonomicProfile.wpi(4, 8, 3, 6);
        List<TimingConstraint> constraints = List.of(
                new DiamondConstraint(m_log, 2, 2, 2),
                new ConstantConstraint(m_log, 1, 1),
                new YawRateConstraint(m_log, 1, 1));
        m_planner = new TrajectoryPlanner(constraints);
        m_viz = new TrajectoryVisualization(fieldLogger);

        m_controller = ControllerFactoryR3.byIdentity(m_log);

        MoveAndHold knight_move = new VelocityFeedforwardOnly(m_log, m_profile, KNIGHT_MOVE, m_drive);
        m_autonChooser.add("knight_move",
                new AnnotatedCommand(knight_move.until(knight_move::isDone).withName("auto knight_move"), null, null));

        /*
         * MoveAndHold one = new VelocityFeedforwardOnly(m_profile, ONE, m_drive);
         * m_autonChooser.add("one",
         * new AnnotatedCommand(one.until(one::isDone).withName("auto one"), null,
         * null));
         * 
         * MoveAndHold two = new VelocityFeedforwardOnly(log, m_profile, TWO, m_drive);
         * m_autonChooser.add("two",
         * new AnnotatedCommand(two.until(two::isDone).withName("auto two"), null,
         * null));
         * 
         * Command three = m_drive.driveWithGlobalVelocity(
         * new GlobalVelocityR3(1.5, 0, 0)).withTimeout(1.0);
         * m_autonChooser.add("three",
         * new AnnotatedCommand(three.withName("auto three"), null, null));
         * 
         * MoveAndHold four = new DriveToPoseWithProfile(
         * log, drive, controller, m_profile, () -> FOUR);
         * m_autonChooser.add("four",
         * new AnnotatedCommand(four.until(four::isDone).withName("auto four"), null,
         * null));
         */

        MoveAndHold knight_l = new DriveWithTrajectoryFunction(
                m_log, drive, m_controller, m_viz, this::knight_l);
        m_autonChooser.add("knight left",
                new AnnotatedCommand(knight_l.until(knight_l::isDone).withName("auto knight_l"), null, null));

        MoveAndHold calib = new DriveWithTrajectoryFunction(
                m_log, drive, m_controller, m_viz, this::calib);
        m_autonChooser.add("calibration",
                new AnnotatedCommand(calib.until(calib::isDone).withName("auto calib"), null, null));

        MoveAndHold autolow = new DriveWithTrajectoryFunction(
                m_log, drive, m_controller, m_viz, this::autolow);
        m_autonChooser.add("straight low auto",
                new AnnotatedCommand(
                        autolow.until(autolow::isDone).andThen(pivot.extend().withTimeout(1)).withName("auto autolow"),
                        null, null));
        
        MoveAndHold musterAutolow = new DriveWithTrajectoryFunction(
            log, drive, controller, m_viz, this::musterAutolowPart1);
        m_autonChooser.add("musterLine low auto",
            new AnnotatedCommand(
                    musterAutolow.until(musterAutolow::isDone).andThen(pivot.extend().withTimeout(1)).withName("auto musterAutolow"),
                    null, null));

        MoveAndHold blue_side_auto_long = new DriveWithTrajectoryFunction(
                m_log, drive, m_controller, m_viz, this::blue_side_auto_long);
        m_autonChooser.add("swerve auto (long)",
                new AnnotatedCommand(
                        blue_side_auto_long.until(blue_side_auto_long::isDone)
                                .andThen(pivot.extend().withTimeout(1))
                                .withName("auto sideautolong"),
                        Alliance.Blue, null));

        MoveAndHold blue_side_auto_short = new DriveWithTrajectoryFunction(
                m_log, drive, m_controller, m_viz, this::blue_side_auto_short);
        m_autonChooser.add("swerve auto (short)",
                new AnnotatedCommand(
                        blue_side_auto_short.until(blue_side_auto_short::isDone)
                                .andThen(pivot.extend().withTimeout(1))
                                .withName("auto sideautoshort"),
                        Alliance.Blue, null));
    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }

    private Trajectory100 knight_l(Pose2d p) {
        Pose2d end = new Pose2d(p.getX() + 2, p.getY() + 1, p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, Math.PI / 2)));
    }

    private Trajectory100 calib(Pose2d p) {
        Pose2d end = new Pose2d(p.getX(), p.getY() + 1, p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, Math.PI / 2)));
    }

    private Trajectory100 autolow(Pose2d p) {
        Pose2d end = new Pose2d(p.getX() + 2.2, p.getY(), p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(end, 0))); // Math.PI / 2
    }
    private Trajectory100 musterAutolowPart1(Pose2d p) {
        Pose2d point_1 = new Pose2d(p.getX() + 2.2, p.getY(), p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(point_1, 0)));
    }

/*    
    private Trajectory100 musterAutolowPart2(Pose2d p) {
        Pose2d point_1 = new Pose2d(p.getX() - 0.125, p.getY(), p.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(point_1, 0)));
    }
*/

    private Trajectory100 blue_side_auto_long(Pose2d p) {
        Pose2d point_1 = new Pose2d(p.getX() + 0.50, p.getY(), p.getRotation());
        Pose2d point_2 = new Pose2d(point_1.getX() + 0.5, point_1.getY() + 2.2, point_1.getRotation());
        Pose2d point_3 = new Pose2d(point_2.getX() + 3.0, point_2.getY(), point_2.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(point_1, 0),
                HolonomicPose2d.make(point_2, Math.PI / 2),
                HolonomicPose2d.make(point_3, 0)));
    }

    private Trajectory100 blue_side_auto_short(Pose2d p) {
        Pose2d point_1 = new Pose2d(p.getX() + 0.50, p.getY(), p.getRotation());
        Pose2d point_2 = new Pose2d(point_1.getX() + 0.5, point_1.getY() + 0.5, point_1.getRotation());
        Pose2d point_3 = new Pose2d(point_2.getX() + 3.0, point_2.getY(), point_2.getRotation());
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(point_1, 0),
                HolonomicPose2d.make(point_2, Math.PI / 2),
                HolonomicPose2d.make(point_3, 0)));
    }

    /*
     * private Trajectory100 swerve(Pose2d p) {
     * Pose2d end = new Pose2d(p.getX(), p.getY() - 1, p.getRotation());
     * return m_planner.restToRest(List.of(
     * HolonomicPose2d.make(p, 0),
     * HolonomicPose2d.make(end, Math.PI / 2)));
     * }
     */

    private Command redRight() {
        LoggerFactory log = m_log.name("red right");
        DriveWithTrajectoryFunction cmd = new DriveWithTrajectoryFunction(
                log,
                m_drive,
                m_controller,
                m_viz,
                (p) -> m_planner.restToRest(List.of(
                        HolonomicPose2d.make(MechanicalMayhem2025.START_RED_RIGHT, 0),
                        HolonomicPose2d.make(MechanicalMayhem2025.START_RED_RIGHT
                                .plus(new Transform2d(1, 1, Rotation2d.kCCW_90deg)), 0))));
        return cmd.until(cmd::isDone).withName("red right");
    }

}
