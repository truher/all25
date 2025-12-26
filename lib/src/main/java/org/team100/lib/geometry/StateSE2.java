package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;

/** For the scheduler. */
public record StateSE2(Pose2d pose, VelocitySE2 vel) {

}
