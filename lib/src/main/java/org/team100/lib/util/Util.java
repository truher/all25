package org.team100.lib.util;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Util {

    public static boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }

    /** This exists to make it clear which println statements to keep. */
    public static void println(Object s) {
        System.out.println(s);
    }

    public static void printf(String s, Object... args) {
        System.out.printf(s, args);
    }

    static void print(Pose3d p) {
        System.out.printf("x %6.3f y %6.3f z %6.3f r %6.3f p %6.3f y %6.3f\n",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    static <R extends Num> void print(String name, Vector<R> v) {
        System.out.printf("%s ", name);
        for (int i = 0; i < v.getNumRows(); ++i) {
            System.out.printf("%6.3f ", v.get(i));
        }
        System.out.println();
    }

    public static String poseStr(Pose3d p) {
        return String.format("%f %f %f %f %f %f",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    public static String rotStr(Rotation3d r) {
        return String.format("%f %f %f",
                r.getX(), r.getY(), r.getZ());
    }

    /**
     * Print to the console.
     */
    public static void warn(String s) {
        System.out.println("WARNING: " + s);
    }

    public static void warnf(String s, Object... args) {
        System.out.printf("WARNING: " + String.format(s, args));
    }

    /** Throw if x is out of range. This is a more strict version of "clamp" :-) */
    public static double inRange(double x, double minX, double maxX) {
        if (x < minX)
            throw new IllegalArgumentException(String.format("arg is %f which is below %f", x, minX));
        if (x > maxX)
            throw new IllegalArgumentException(String.format("arg is %f which is above %f", x, maxX));
        return x;
    }

    public static double notNaN(double x) {
        if (Double.isNaN(x))
            throw new IllegalArgumentException("arg is NaN");
        return x;
    }

    private Util() {
        //
    }
}
