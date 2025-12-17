package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.timing.ScheduleGenerator;

/**
 * Represents a 2d holonomic path, i.e. with heading independent from course.
 * 
 * There's no timing information here. For that, see Trajectory100.
 */
public class Path100 {
    private final List<Pose2dWithMotion> m_points;
    /** double geodesic distance which is meters and radians */
    private final double[] m_distances;

    public Path100(final List<Pose2dWithMotion> states) {
        m_points = new ArrayList<>(states.size());
        m_distances = new double[states.size()];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i = 1; i < states.size(); ++i) {
            m_points.add(states.get(i));
            Pose2dWithMotion p0 = getPoint(i - 1);
            Pose2dWithMotion p1 = getPoint(i);
            // use the distance metric that includes rotation
            double dist = GeometryUtil.doubleGeodesicDistance(p0, p1);
            m_distances[i] = m_distances[i - 1] + dist;
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public Pose2dWithMotion getPoint(int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }

    /** This is always non-negative. */
    public double getMaxDistance() {
        if (m_points.isEmpty())
            return 0.0;
        return m_distances[m_distances.length - 1];
    }

    /**
     * Walks through all the points to find the bracketing points.
     * 
     * @param distance in meters, always a non-negative number.
     */
    public Pose2dWithMotion sample(double distance) throws ScheduleGenerator.TimingException {
        if (distance >= getMaxDistance()) {
            // off the end
            return getPoint(length() - 1);
        }
        if (distance <= 0.0) {
            // before the start
            return getPoint(0);
        }
        for (int i = 1; i < length(); ++i) {
            // walk through the points to bracket the desired distance
            Pose2dWithMotion p0 = getPoint(i - 1);
            Pose2dWithMotion p1 = getPoint(i);
            double d0 = m_distances[i - 1];
            double d1 = m_distances[i];
            double d = d1 - d0;
            if (d1 >= distance) {
                // Found the bracket.
                double s = (distance - d0) / d;
                return p0.interpolate(p1, s);
            }
        }
        throw new ScheduleGenerator.TimingException();
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }
}
