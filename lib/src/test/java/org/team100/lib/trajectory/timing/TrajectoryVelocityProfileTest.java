package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.timing.ScheduleGenerator.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Verify that trajectory schedule generation yields a realistic profile.
 * 
 * see
 * https://docs.google.com/spreadsheets/d/16UUCCz-qcPz_YZMnsJnVkVO1KGp5zHCOVo7EoJct2nA/edit?gid=0#gid=0
 */
public class TrajectoryVelocityProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // A five-meter straight line.
    public static final Pose2dWithMotion[] WAYPOINTS = new Pose2dWithMotion[] {
            new Pose2dWithMotion(WaypointSE2.irrotational(
                    new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new Pose2dWithMotion(WaypointSE2.irrotational(
                    new Pose2d(2.5, 0, new Rotation2d(0)), 0, 1.2), 0, 0),
            new Pose2dWithMotion(WaypointSE2.irrotational(
                    new Pose2d(5, 0, new Rotation2d(0)), 0, 1.2), 0, 0) };

    // No rotation.
    public static final List<Rotation2d> HEADINGS = List.of(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0));

    void print(Trajectory100 traj) {
        if (!DEBUG)
            return;
        System.out.println("t, v");
        for (double t = 0; t < traj.duration(); t += 0.02) {
            System.out.printf("%6.3f, %6.3f\n", t, traj.sample(t).velocityM_S());
        }

    }

    /**
     * This uses the default max accel which is ridiculously high.
     */
    @Test
    void testNoConstraint() throws TimingException {
        List<TimingConstraint> constraints = new ArrayList<TimingConstraint>();
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(WAYPOINTS, 0, 0);
        print(traj);
    }

    /**
     * This produces a trapezoid.
     */
    @Test
    void testConstantConstraint() throws TimingException {
        // somewhat realistic numbers
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(logger, 1, 1, limits));
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(WAYPOINTS, 0, 0);
        print(traj);
    }

    /**
     * This produces the desired current-limited exponential shape.
     */
    @Test
    void testSwerveConstraint() throws TimingException {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        List<TimingConstraint> constraints = List.of(new SwerveDriveDynamicsConstraint(logger, limits, 1, 1));
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(WAYPOINTS, 0, 0);
        print(traj);
    }

    @Test
    void testAuto() throws TimingException {
        // i think this is broken because one of the constraints
        // is producing zero as the acceleration limit
        // even though the velocity is also zero
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest(logger);
        TimingConstraintFactory timing = new TimingConstraintFactory(limits);
        List<TimingConstraint> constraints = timing.testAuto(logger);
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(
                new Path100(Arrays.asList(WAYPOINTS)), 0.5, 0, 0);
        // Trajectory100 traj = u.timeParameterizeTrajectory(WAYPOINTS, 0, 0);
        print(traj);
    }
}
