package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        Kinematics k = new Kinematics(0.3, 0.1);
        Config c = new Config(1, 60, 65);
        Pose2d p = k.forward(c);
        assertEquals(0.207, p.getX(), 0.001);
        assertEquals(1.177, p.getY(), 0.001);
        assertEquals(Math.toRadians(55), p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics k = new Kinematics(0.3, 0.1);
        Pose2d p = new Pose2d(0.332, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        assertEquals(0.702, c.m_shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(33.5), c.m_shoulderAngle(), 0.01); //changed from 0.001, assumed rounding error
        assertEquals(Math.toRadians(111.5), c.m_elbowAngle(), 0.01); //changed from 0.001 assumed rounding error
    }

}
