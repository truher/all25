package org.team100.lib.subsystems.lynxmotion_arm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Position of each joint.
 */
public record TwoDofArmPosition(Translation2d p1, Translation2d p2) {
}