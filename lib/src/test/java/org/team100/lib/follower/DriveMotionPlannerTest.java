package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.MockDrive;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathFactory;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.ScheduleGenerator;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class DriveMotionPlannerTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testFieldRelativeTrajectory() {
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(100, 4), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(196, 13), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

        double start_vel = 0.0;
        double end_vel = 0.0;

        Path100 path = PathFactory.pathFromWaypoints(waypoints, 2, 0.25, 0.1);
        assertFalse(path.isEmpty());

        double stepSize = 2;
        ScheduleGenerator u = new ScheduleGenerator(Arrays.asList());
        Trajectory100 trajectory = u.timeParameterizeTrajectory(
                path,
                stepSize,
                start_vel,
                end_vel);
        if (DEBUG)
            Util.printf("TRAJECTORY:\n%s\n", trajectory);

        FullStateSwerveController swerveController = new FullStateSwerveController(
                logger,
                2.4, 2.4,
                0.1, 0.1,
                0.01, 0.02,
                0.01, 0.02);

        MockDrive drive = new MockDrive();
        drive.m_state = new SwerveModel();
        ReferenceController referenceController = new ReferenceController(
                drive,
                swerveController,
                new TrajectoryReference(trajectory),
                false);

        Pose2d pose = trajectory.sample(0).state().getPose();
        FieldRelativeVelocity velocity = FieldRelativeVelocity.zero();

        double mDt = 0.02;
        int i = 0;
        while (!referenceController.isDone()) {
            if (++i > 500)
                break;
            stepTime();
            drive.m_state = new SwerveModel(pose, velocity);
            referenceController.execute();
            velocity = drive.m_recentSetpoint;
            pose = new Pose2d(
                    pose.getX() + velocity.x() * mDt,
                    pose.getY() + velocity.y() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta() * mDt));
            if (DEBUG)
                Util.printf("pose %s vel %s\n", pose, velocity);
        }

        // this should be exactly right but it's not.
        assertEquals(195, pose.getTranslation().getX(), 1);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.1);
    }
}