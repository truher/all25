package org.team100.frc2025;

import java.util.List;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.field.MechanicalMayhem2025;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.shooter.IndexerServo;
import org.team100.lib.subsystems.shooter.SingleDrumShooter;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.commands.FixedTrajectory;
import org.team100.lib.subsystems.tank.commands.ToPoseWithTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The auton selector is just a flat list.
 * 
 * Each item contains:
 * 
 * * a command
 * * a starting position
 * * an alliance color
 * 
 * These should be checked against the true color and true position, and an
 * alert should be raised if they're wrong.
 * 
 * If a command works for arbitrary position, leave it null.
 * 
 * If a command works for either alliance, leave it null.
 * 
 * The name should be fully descriptive, i.e. contain something about the
 * position and color if that's required to choose it in the dashboard.
 */
public class Autons {
    private final LoggerFactory m_log;
    private final TankDrive m_drive;
    private final IndexerServo m_indexer;
    private final SingleDrumShooter m_shooter;
    private final TrajectoryVisualization m_trajectoryViz;
    private final AutonChooser m_autonChooser;

    public Autons(
            LoggerFactory log,
            TankDrive drive,
            IndexerServo indexer,
            SingleDrumShooter shooter,
            TrajectoryVisualization trajectoryViz) {
        m_log = log.type(this);
        m_drive = drive;
        m_indexer = indexer;
        m_shooter = shooter;
        m_trajectoryViz = trajectoryViz;

        m_autonChooser = new AutonChooser();

        m_autonChooser.add("red left",
                new AnnotatedCommand(redLeft(), Alliance.Red, MechanicalMayhem2025.START_RED_LEFT));

        m_autonChooser.add("blue left",
                new AnnotatedCommand(blueLeft(), Alliance.Blue, MechanicalMayhem2025.START_BLUE_LEFT));

        m_autonChooser.add("red center",
                new AnnotatedCommand(redCenter(), Alliance.Red, MechanicalMayhem2025.START_RED_CENTER));

        m_autonChooser.add("blue center",
                new AnnotatedCommand(blueCenter(), Alliance.Blue, MechanicalMayhem2025.START_BLUE_CENTER));

        m_autonChooser.add("red right",
                new AnnotatedCommand(redRight(), Alliance.Red, MechanicalMayhem2025.START_RED_RIGHT));

        m_autonChooser.add("blue right",
                new AnnotatedCommand(blueRight(), Alliance.Blue, MechanicalMayhem2025.START_BLUE_RIGHT));

    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }

    private Trajectory100 redLeftTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_LEFT),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_LEFT
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Trajectory100 redRightTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_RIGHT),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_RIGHT
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Trajectory100 redCenterTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_CENTER),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_RED_CENTER
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Trajectory100 blueLeftTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_LEFT),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_LEFT
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Trajectory100 blueRightTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_RIGHT),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_RIGHT
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Trajectory100 blueCenterTrajectory(TrajectoryPlanner planner) {
        // delaying construction allows trajectory constraints to be mutable
        return planner.restToRest(List.of(
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_CENTER),
                HolonomicPose2d.tank(MechanicalMayhem2025.START_BLUE_CENTER
                        .plus(new Transform2d(1.5, 0, Rotation2d.kZero)))));
    }

    private Command redLeft() {
        LoggerFactory log = m_log.name("red left");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> redLeftTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }

    private Command redRight() {
        LoggerFactory log = m_log.name("red right");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> redRightTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }

    private Command redCenter() {
        LoggerFactory log = m_log.name("red center");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> redCenterTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }

    private Command blueRight() {
        LoggerFactory log = m_log.name("blue right");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> blueRightTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }

    private Command blueLeft() {
        LoggerFactory log = m_log.name("blue left");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> blueLeftTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }

    private Command blueCenter() {
        LoggerFactory log = m_log.name("blue center");
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(log, 1, 1)));
        FixedTrajectory cmd = new FixedTrajectory(
                () -> blueCenterTrajectory(planner),
                m_drive,
                m_trajectoryViz);
        return cmd
                .until(cmd::isDone)
                .andThen(
                        new SingleShoot(m_shooter, m_indexer)
                                .withTimeout(1));
    }
}