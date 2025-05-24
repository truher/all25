package org.team100.lib.motion.urdf;

import java.util.List;

import org.team100.lib.motion.urdf.URDFModel.Joint;
import org.team100.lib.motion.urdf.URDFModel.Joint.JointType;
import org.team100.lib.motion.urdf.URDFModel.Joint.Limit;
import org.team100.lib.motion.urdf.URDFModel.Link;
import org.team100.lib.motion.urdf.URDFModel.Robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Cartesian prismatic, like the Shopsabre:
 * 
 * * base is the table at the origin
 * * gantry moves in y
 * * headstock moves in x (along the gantry)
 * * spindle moves in z (along the headstock)
 * 
 * This should be very easy to solve :-)
 */
public class URDFCartesian {
    public static final Robot ROBOT;
    static {
        Link base = new Link("base");
        Link gantry = new Link("gantry");
        Link head_stock = new Link("head_stock");
        Link spindle = new Link("spindle");
        Link tool_center_point = new Link("tool_center_point");
        ROBOT = new Robot(
                "Cartesian",
                List.of(
                        base,
                        gantry,
                        head_stock,
                        spindle,
                        tool_center_point),
                List.of(
                        new Joint(
                                "base_gantry",
                                JointType.prismatic,
                                new Limit(1000, 0, 1, 1),
                                base,
                                gantry,
                                new Pose3d(),
                                VecBuilder.fill(0, 1, 0)),
                        new Joint(
                                "gantry_head",
                                JointType.prismatic,
                                new Limit(1000, 0, 1, 1),
                                gantry,
                                head_stock,
                                new Pose3d(),
                                VecBuilder.fill(1, 0, 0)),
                        new Joint(
                                "head_spindle",
                                JointType.prismatic,
                                new Limit(1000, 0, 0.1, 1),
                                head_stock,
                                spindle,
                                new Pose3d(),
                                VecBuilder.fill(0, 0, 1)),
                        new Joint(
                                "center_point",
                                JointType.fixed,
                                null,
                                spindle,
                                tool_center_point,
                                new Pose3d(0, 0, -0.1, new Rotation3d()),
                                null)));
    }
}
