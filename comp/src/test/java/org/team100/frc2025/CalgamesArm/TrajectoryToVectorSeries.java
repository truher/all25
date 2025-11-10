package org.team100.frc2025.CalgamesArm;

import org.jfree.data.xy.VectorSeries;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryToVectorSeries {
    /** Time between samples */
    private static final double DT = 0.1;

    /** Length of the vector indicating heading */
    private final double m_scale;

    public TrajectoryToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public VectorSeries convert(Trajectory100 t) {
        VectorSeries s = new VectorSeries("trajectory");
        double duration = t.duration();
        for (double time = 0; time < duration; time += DT) {
            TimedPose p = t.sample(time);
            HolonomicPose2d pp = p.state().getPose();
            double x = pp.translation().getX();
            double y = pp.translation().getY();
            Rotation2d heading = pp.heading();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return s;
    }

    /** Maps x to y, y to -x */
    public VectorSeries convertRotated(Trajectory100 t) {
        VectorSeries s = new VectorSeries("trajectory");
        double duration = t.duration();
        for (double time = 0; time < duration; time += DT) {
            TimedPose p = t.sample(time);
            HolonomicPose2d pp = p.state().getPose();
            double y = pp.translation().getX();
            double x = -pp.translation().getY();
            Rotation2d heading = pp.heading();
            double dy = m_scale * heading.getCos();
            double dx = -m_scale * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return s;
    }

}
