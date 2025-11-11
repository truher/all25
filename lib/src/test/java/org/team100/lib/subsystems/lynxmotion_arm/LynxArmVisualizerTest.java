package org.team100.lib.subsystems.lynxmotion_arm;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class LynxArmVisualizerTest {
    private static final boolean DEBUG = false;

    @Test
    void testFoo() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();
        try (LynxArm arm = new LynxArm(kinematics)) {
            LynxArmVisualizer foo = new LynxArmVisualizer(arm::getPosition);
            // unrotated camera below the table
            Pose3d cameraPose = new Pose3d(0, 0, -4, new Rotation3d(0, 0, 0));
            // a cube centered on the origin
            List<Pose3d> tList = List.of(
                    new Pose3d(1, 1, 1, Rotation3d.kZero),
                    new Pose3d(-1, 1, 1, Rotation3d.kZero),
                    new Pose3d(-1, -1, 1, Rotation3d.kZero),
                    new Pose3d(1, -1, 1, Rotation3d.kZero),
                    new Pose3d(1, 1, -1, Rotation3d.kZero),
                    new Pose3d(-1, 1, -1, Rotation3d.kZero),
                    new Pose3d(-1, -1, -1, Rotation3d.kZero),
                    new Pose3d(1, -1, -1, Rotation3d.kZero));
            MatOfPoint2f points = foo.project(cameraPose, tList);
            List<Point> pointList = points.toList();
            if (DEBUG)
                System.out.printf("pointList.size() = %d\n", pointList.size());
            for (Point p : pointList) {
                if (DEBUG)
                    System.out.printf("%5.3f, %5.3f\n", p.x, p.y);
            }
        }
    }

}
