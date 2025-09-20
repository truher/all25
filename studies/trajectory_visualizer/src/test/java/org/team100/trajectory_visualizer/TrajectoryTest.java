package org.team100.trajectory_visualizer;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryTest {

    /**
     * Yields a straight line.
     * 
     * TrajectoryPlanner.restToRest() has several overloads: the one that takes
     * two non-holonomic poses draws a straight line between them.
     */
    @Test
    void testSimple() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 0.1),
                new YawRateConstraint(1, 1));
        TrajectoryPlanner p = new TrajectoryPlanner(c);
        Trajectory100 t = p.restToRest(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(10, 1, new Rotation2d()));
        TrajectoryPlotter.plot(t, "simple");
    }

    /**
     * Yields a curve.
     * 
     * A HolonomicPose2d allows separate specification of heading (which way the
     * front of the robot is facing) and course (which way the robot is moving).
     * 
     * In this case, is facing +x, and moving +x, and it ends up moving +y but
     * facing the other way (i.e. backwards)
     */
    @Test
    void testCurved() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectoryPlanner p = new TrajectoryPlanner(c);
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(9, 9), new Rotation2d(-Math.PI / 2),
                        new Rotation2d(Math.PI / 2)));
        Trajectory100 t = p.restToRest(waypoints);
        TrajectoryPlotter.plot(t, "simple");
    }

    /**
     * You can specify interior waypoints as well as start and end points. Note that
     * specifying many such points will make the curve harder to calculate and
     * harder to make smooth.
     */
    @Test
    void testMultipleWaypoints() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectoryPlanner p = new TrajectoryPlanner(c);
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(
                        new Translation2d(1, 1), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(
                        new Translation2d(5, 5), new Rotation2d(-2), new Rotation2d()),
                new HolonomicPose2d(
                        new Translation2d(9, 9), new Rotation2d(-Math.PI / 2), new Rotation2d(Math.PI / 2)));
        Trajectory100 t = p.restToRest(waypoints);
        TrajectoryPlotter.plot(t, "simple");
    }

    @Test
    void testPickupToPlace() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectoryPlanner p = new TrajectoryPlanner(c);
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(
                        new Translation2d(1, 0.1), new Rotation2d(-Math.PI), new Rotation2d(Math.PI/2)), //pickup
                new HolonomicPose2d(
                        new Translation2d(3, 7), new Rotation2d(Math.PI/2), new Rotation2d()), //place for gateway point?
                new HolonomicPose2d(
                        new Translation2d(6, 9), new Rotation2d(-((7*Math.PI)/36)), new Rotation2d(Math.PI / 2)));
        Trajectory100 t = p.restToRest(waypoints);
        TrajectoryPlotter.plot(t, "simple");    
    }

    @Test
    void testSingularityDemo() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectoryPlanner p = new TrajectoryPlanner(c);
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(
                        new Translation2d(1, 0.1), new Rotation2d(-Math.PI), new Rotation2d(Math.PI/2)), //pickup
                new HolonomicPose2d(
                        new Translation2d(0.75, 3), new Rotation2d(-Math.PI), new Rotation2d(Math.PI/2)), //place for gateway point
                new HolonomicPose2d(
                        new Translation2d(3, 7), new Rotation2d(Math.PI/2), new Rotation2d()), //place for gateway point
                new HolonomicPose2d(
                        new Translation2d(6, 9), new Rotation2d(-((7*Math.PI)/36)), new Rotation2d(Math.PI / 2)));
        Trajectory100 t = p.restToRest(waypoints);
        TrajectoryPlotter.plot(t, "simple");    
    }

    /** Example of using a trajectory library. */
    @Test
    void testExample() {
        TrajectoryPlotter.plot(TrajectoryExample.oneToTwo(), "library example");
    }

}
