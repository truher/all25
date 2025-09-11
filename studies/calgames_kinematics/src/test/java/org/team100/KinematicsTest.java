package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        Kinematics k = new Kinematics(3, 2, 5, 5, 60);
        Config c = new Config(0, 0, 0);
        Pose2d p = k.forward(c);
        assertEquals(1, p.getX(), 0.001);
        assertEquals(1, p.getY(), 0.001);
        assertEquals(1, p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics k = new Kinematics(3, 2, 5, 5, 60);
        Pose2d p = new Pose2d(0, 0, new Rotation2d());
        Config c = k.inverse(p);
        assertEquals(1, c.m_shoulderHeight(), 0.001);
        assertEquals(1, c.m_shoulderAngle(), 0.001);
        assertEquals(1, c.m_elbowAngle(), 0.001);
    }

}
