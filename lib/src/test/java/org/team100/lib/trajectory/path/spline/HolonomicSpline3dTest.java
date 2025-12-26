package org.team100.lib.trajectory.path.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE3;
import org.team100.lib.geometry.Pose3dWithDirection;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class HolonomicSpline3dTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testLinear() {
        HolonomicSpline3d s = new HolonomicSpline3d(
                new Pose3dWithDirection(
                        new Pose3d(
                                new Translation3d(),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0)),
                new Pose3dWithDirection(
                        new Pose3d(
                                new Translation3d(1, 0, 0),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0)));
        Translation3d t = s.getPoint(0);
        assertEquals(0, t.getX(), DELTA);
        t = s.getPoint(1);
        assertEquals(1, t.getX(), DELTA);

        // Pose3dWithMotion p = s.getPose3dWithMotion(0);
        // assertEquals(0, p.getPose().translation().getX(), DELTA);
        // assertEquals(0, p.getPose().heading().getZ(), DELTA);
        // assertEquals(0, p.getHeadingYawRateRad_M(), DELTA);

        // p = s.getPose3dWithMotion(1);
        // assertEquals(1, p.getPose().translation().getX(), DELTA);
        // assertEquals(0, p.getPose().heading().getZ(), DELTA);
        // assertEquals(0, p.getHeadingYawRateRad_M(), DELTA);
    }

}
