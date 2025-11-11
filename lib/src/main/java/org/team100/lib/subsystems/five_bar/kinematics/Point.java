package org.team100.lib.subsystems.five_bar.kinematics;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import java.util.OptionalDouble;

/** A cartesian (x,y) point. */
public record Point(double x, double y) {

    public double distance(Point other) {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    /** Radians. Empty at the origin */
    public OptionalDouble angle() {
        if (norm() < 1e-6)
            return OptionalDouble.empty();
        return OptionalDouble.of(Math.atan2(y, x));
    }

    public Point plus(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point minus(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point times(double scale) {
        return new Point(x * scale, y * scale);
    }

    public Point rotateBy(double a) {
        double c = Math.cos(a);
        double s = Math.sin(a);
        return new Point(x * c - y * s, x * s + y * c);
    }

    public boolean valid() {
        if (Double.isNaN(x))
            return false;
        if (Double.isNaN(y))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return String.format("(%6.3f %6.3f)", x, y);
    }
}
