package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.ScheduleGenerator.TimingException;
import org.team100.lib.trajectory.timing.TimedPose;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

class Trajectory100Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testPreviewAndAdvance() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest(logger);
        Pose2d start = Pose2d.kZero;
        Pose2d end = start.plus(new Transform2d(1, 0, Rotation2d.kZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(start, DirectionSE2.irrotational(angleToGoal), 1),
                new WaypointSE2(end, DirectionSE2.irrotational(angleToGoal), 1));

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast(logger);
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypoints);

        TimedPose sample = trajectory.sample(0);
        assertEquals(0, sample.state().getPose().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(1);
        assertEquals(1, sample.state().getPose().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(2);
        assertEquals(1, sample.state().getPose().pose().getTranslation().getX(), DELTA);

        sample = trajectory.sample(3);
        assertEquals(1, sample.state().getPose().pose().getTranslation().getX(), DELTA);
    }

    @Test
    void testSample() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3(logger);
        Pose2d start = Pose2d.kZero;
        Pose2d end = start.plus(new Transform2d(1, 0, Rotation2d.kZero));

        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(start, DirectionSE2.irrotational(angleToGoal), 1),
                new WaypointSE2(end, DirectionSE2.irrotational(angleToGoal), 1));

        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast(logger);
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypoints);

        assertEquals(1.417, trajectory.duration(), DELTA);
        TimedPose sample = trajectory.sample(0);
        assertEquals(0, sample.state().getPose().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(1);
        assertEquals(0.825, sample.state().getPose().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(2);
        assertEquals(1, sample.state().getPose().pose().getTranslation().getX(), DELTA);
    }

    /**
     * This is to hold the sample invariant while adding the index.
     */
    @Test
    void testSampleThoroughly() {

        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1));

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast(logger);
        TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

        Trajectory100 trajectory = planner.restToRest(waypoints);

        assertEquals(1.417, trajectory.duration(), DELTA);
        assertEquals(0.000, trajectory.sample(0.0).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.010, trajectory.sample(0.1).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.040, trajectory.sample(0.2).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.090, trajectory.sample(0.3).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.160, trajectory.sample(0.4).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.250, trajectory.sample(0.5).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.360, trajectory.sample(0.6).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.487, trajectory.sample(0.7).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.618, trajectory.sample(0.8).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.732, trajectory.sample(0.9).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.825, trajectory.sample(1.0).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.899, trajectory.sample(1.1).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.953, trajectory.sample(1.2).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.987, trajectory.sample(1.3).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(1.000, trajectory.sample(1.4).state().getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(1.000, trajectory.sample(1.5).state().getPose().pose().getTranslation().getX(), DELTA);
    }

    /**
     * Does the index help? No.
     * 
     * Does interpolation help, relative to just sampling the spline directly?  No.
     * 
     * There's no need to run this all the time
     */
    @Test
    void testSamplePerformance() throws TimingException {
        WaypointSE2 p0 = new WaypointSE2(new Pose2d(new Translation2d(), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 p1 = new WaypointSE2(new Pose2d(new Translation2d(10, 0), Rotation2d.kCCW_Pi_2),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 p2 = new WaypointSE2(new Pose2d(new Translation2d(10, 10), Rotation2d.kPi),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(p0, p1, p2);

        int reps = 100000;
        int times = 10;

        // SAMPLE SPLINE DIRECTLY (200 ns)

        HolonomicSpline spline = new HolonomicSpline(p0, p1);

        long start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                spline.getPose2dWithMotion(0.1 * t);
            }
        }
        long end = System.nanoTime();
        long duration = end - start;
        if (DEBUG)
            System.out.printf("SPLINE duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

        // INTERPOLATE SPLINE POINTS (170 ns)

        Path100 path = PathFactory.pathFromWaypoints(
                waypoints, 0.02, 0.2, 0.1);
        assertEquals(23.063, path.getMaxDistance(), 0.001);

        start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                path.sample(0.1 * t);
            }
        }
        end = System.nanoTime();
        duration = end - start;
        if (DEBUG)
            System.out.printf("PATH duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

        /////////////
        // INTERPOLATE TRAJECTORY POINTS (335 ns)

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest3(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(limits).fast(logger);
        ScheduleGenerator generator = new ScheduleGenerator(constraints);

        Trajectory100 trajectory = generator.timeParameterizeTrajectory(path, 0.1, 0, 0);

        assertEquals(232, trajectory.length());

        start = System.nanoTime();
        for (int rep = 0; rep < reps; ++rep) {
            for (int t = 0; t < times; ++t) {
                trajectory.sample(0.1 * t);
            }
        }
        end = System.nanoTime();
        duration = end - start;
        if (DEBUG)
            System.out.printf("TRAJECTORY duration (ns) total (ms) %.0f per sample (ns) %.2f\n",
                    0.000001 * duration, (double) duration / (reps * times));

    }

}
