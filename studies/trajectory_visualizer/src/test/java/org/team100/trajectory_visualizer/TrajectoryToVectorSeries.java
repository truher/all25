package org.team100.trajectory_visualizer;

import org.jfree.data.xy.VectorSeries;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryToVectorSeries {
    /** Time between samples */
    private static final double DT = 0.1;
    /** Length of the vector indicating heading */
    private static final double SCALE = 1;

    public static VectorSeries convert(Trajectory100 t) {
        VectorSeries s = new VectorSeries("trajectory");
        double duration = t.duration();
        for (double time = 0; time < duration; time += DT) {
            TimedPose p = t.sample(time);
            Pose2dWithMotion w = p.state();
            Pose2d pp = w.getPose();
            double x = pp.getX();
            double y = pp.getY();
            Rotation2d heading = pp.getRotation();
            double dx = SCALE * heading.getCos();
            double dy = SCALE * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return s;
    }

}
