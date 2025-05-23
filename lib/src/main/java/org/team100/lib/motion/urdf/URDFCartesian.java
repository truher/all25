package org.team100.lib.motion.urdf;

import java.util.List;

import org.team100.lib.motion.urdf.URDFModel.Joint;
import org.team100.lib.motion.urdf.URDFModel.Joint.JointType;
import org.team100.lib.motion.urdf.URDFModel.Joint.Limit;
import org.team100.lib.motion.urdf.URDFModel.Link;
import org.team100.lib.motion.urdf.URDFModel.Robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Cartesian prismatic, like the Shopsabre:
 * * base is the table at the origin
 * * gantry moves in y
 * * headstock moves in x (along the gantry)
 * * spindle moves in z (along the headstock)
 */
public class URDFCartesian {
    public static final Robot ROBOT;
    static {
        Link base = new Link("base");
        Link gantry = new Link("gantry");
        Link head_stock = new Link("head_stock");
        Link spindle = new Link("spindle");

        ROBOT = new Robot(
                "Cartesian",
                List.of(
                        base,
                        gantry,
                        head_stock,
                        spindle),
                List.of(
                    new Joint(
                        "base_gantry",
                        JointType.prismatic,
                        new Limit(1000, 0, 1, 1),
                        base,
                        gantry,
                        new Pose3d(),
                        VecBuilder.fill(0, 1, 0)
                    );
                ));
    }
}
