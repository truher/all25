package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.util.List;

import org.team100.lib.commands.drivetrain.tank.FixedTrajectory;
import org.team100.lib.commands.drivetrain.tank.ToPoseWithTrajectory;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;

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
    private final TankDrive m_drive;

    private final AutonChooser m_autonChooser;
    private final TrajectoryPlanner m_planner;

    public Autons(TankDrive drive) {
        m_drive = drive;
        m_planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(1, 1)));

        m_autonChooser = new AutonChooser();
        m_autonChooser.add("red left",
                new AnnotatedCommand(redLeft(), Alliance.Red, Field.START_RED_LEFT));
        m_autonChooser.add("blue left",
                new AnnotatedCommand(print("blue left"), Alliance.Blue, Field.START_BLUE_LEFT));

        m_autonChooser.add("red center",
                new AnnotatedCommand(print("red center"), Alliance.Red, Field.START_RED_CENTER));
        m_autonChooser.add("blue center",
                new AnnotatedCommand(print("blue center"), Alliance.Blue, Field.START_BLUE_CENTER));

        m_autonChooser.add("red right",
                new AnnotatedCommand(redRight(), Alliance.Red, Field.START_RED_RIGHT));
        m_autonChooser.add("blue right",
                new AnnotatedCommand(print("blue right"), Alliance.Blue, Field.START_BLUE_RIGHT));

    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }

    private Command redLeft() {
        Trajectory100 trajectory = m_planner.restToRest(List.of(
                HolonomicPose2d.tank(Field.START_RED_LEFT),
                HolonomicPose2d.tank(Field.START_RED_LEFT
                        .plus(new Transform2d(1, 1, Rotation2d.kCCW_90deg))),
                HolonomicPose2d.tank(Field.START_RED_LEFT
                        .plus(new Transform2d(2, 2, Rotation2d.kZero)))));
        FixedTrajectory cmd = new FixedTrajectory(trajectory, m_drive);
        return cmd.until(cmd::isDone).withName("red left");
    }

    private Command redRight() {
        ToPoseWithTrajectory cmd = new ToPoseWithTrajectory(
                Field.START_RED_RIGHT
                        .plus(new Transform2d(1, 1, Rotation2d.kCCW_90deg)),
                m_drive);
        return cmd.until(cmd::isDone).withName("red right");
    }
}
