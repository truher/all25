package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.CapsizeAccelerationConstraint;
import org.team100.lib.timing.ConstantConstraint;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.timing.YawRateConstraint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectoryPlannerTest {
    private static final boolean DEBUG = false;
    private static final double kDelta = 0.01;

    /**
     * Stationary trajectories do not work.
     */
    @Test
    void testStationary() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        Trajectory100 t = planner.restToRest(waypoints);
        assertTrue(t.isEmpty());
    }

    @Test
    void testLinear() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        Trajectory100 t = planner.restToRest(waypoints);
        assertEquals(12, t.length());
        TimedPose p = t.getPoint(6);
        assertEquals(0.6, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
    }

    @Test
    void testBackingUp() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, new Rotation2d(Math.PI)),
                new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero, Rotation2d.kZero));
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();

        // these are the same as StraightLineTrajectoryTest.
        List<TimingConstraint> constraints =
                List.of(
                        new ConstantConstraint(1, 1, limits),
                        new SwerveDriveDynamicsConstraint(limits, 1, 1),
                        new YawRateConstraint(limits, 0.2),
                        new CapsizeAccelerationConstraint(limits, 0.2));
        double start_vel = 1;
        double end_vel = 0;
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        Trajectory100 t = planner.generateTrajectory(
                waypoints, start_vel, end_vel);
        TimedPose p = t.getPoint(6);
        assertEquals(0.272, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);

    }

    /**
     * 0.23 ms on my machine.
     * 
     * See PathFactoryTest::testPerformance().
     */
    @Test
    void testPerformance() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        long startTimeNs = System.nanoTime();
        Trajectory100 t = new Trajectory100();
        // for profiling
        // final long iterations = 10000000000l;
        final long iterations = 100l;
        for (long i = 0; i < iterations; ++i) {
            t = planner.restToRest(waypoints);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            Util.printf("total duration ms: %5.3f\n", totalDurationMs);
            Util.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(18, t.length());
        TimedPose p = t.getPoint(6);
        assertEquals(0.575, p.state().getPose().getX(), kDelta);
        assertEquals(0, p.state().getHeadingRate(), kDelta);
    }

    /**
     * Pure rotation does not work.
     */
    @Test
    void testRotation() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(1), new Rotation2d()));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        Trajectory100 t = planner.restToRest(waypoints);
        assertTrue(t.isEmpty());
    }

    @Test
    void testRestToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        SwerveModel start = new SwerveModel(Pose2d.kZero, new FieldRelativeVelocity(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        Trajectory100 trajectory = planner.restToRest(start.pose(), end);
        assertEquals(1.565, trajectory.duration(), kDelta);

        /** progress along trajectory */
        double m_timeS = 0;

        // initial velocity is zero.
        assertEquals(0, trajectory.sample(m_timeS).velocityM_S(), kDelta);

        double maxDriveVelocityM_S = swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxDriveAccelerationM_S2 = swerveKinodynamics.getMaxDriveAccelerationM_S2();
        assertEquals(5, maxDriveVelocityM_S);
        assertEquals(10, maxDriveAccelerationM_S2);
        for (TimedPose p : trajectory.getPoints()) {
            assertTrue(p.velocityM_S() - 0.001 <= maxDriveVelocityM_S,
                    String.format("%f %f", p.velocityM_S(), maxDriveVelocityM_S));
            assertTrue(p.acceleration() - 0.001 <= maxDriveAccelerationM_S2,
                    String.format("%f %f", p.acceleration(), maxDriveAccelerationM_S2));
        }
    }

    @Test
    void testMovingToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        SwerveModel start = new SwerveModel(Pose2d.kZero, new FieldRelativeVelocity(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        Trajectory100 traj = planner.movingToRest(start, end);
        assertEquals(1.176, traj.duration(), kDelta);
    }

    @Test
    void testBackingUp2() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        SwerveModel start = new SwerveModel(Pose2d.kZero, new FieldRelativeVelocity(-1, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        Trajectory100 traj = planner.movingToRest(start, end);
        assertEquals(1.176, traj.duration(), kDelta);
    }

    @Test
    void test2d() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);
        SwerveModel start = new SwerveModel(Pose2d.kZero, new FieldRelativeVelocity(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        Trajectory100 traj = planner.movingToRest(start, end);
        assertEquals(2.525, traj.duration(), kDelta);
    }

}
