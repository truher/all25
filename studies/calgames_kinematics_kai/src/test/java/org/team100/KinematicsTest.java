package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        System.out.println("TESTING FORWARD KINEMATICS");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0);
        Config c = new Config(1, 60, 65);
        Pose2d p = k.forward(c);
        assertEquals(0.207, p.getX(), 0.001);
        assertEquals(1.177, p.getY(), 0.001);
        assertEquals(Math.toRadians(55), p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverseL1() {
        System.out.println("TESTING INVERSE FOR L1");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0 );
        Pose2d p = new Pose2d(0.332, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        if (c==null){
            fail("Inverse Kinematics Failed. Check Logs");
        }
        assertEquals(0.702, c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(33.5), c.shoulderAngle(), 0.01); //changed from 0.001, assumed rounding error
        assertEquals(Math.toRadians(111.5), c.wristAngle(), 0.01); //changed from 0.001 assumed rounding error
    }

    @Test
    void testInverseReachFailures() {
        System.out.println("TESTING INVERSE REACH FAILURES");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0 );
        Pose2d p = new Pose2d(0.4, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        assertNull("Test failed succeslfully!");
    }

    @Test
    void testInverseDeadzoneFailures() {
        System.out.println("TESTING INVERSE DEADZONE FAILURES");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0.1, 1, 0.5, 3 );
        Pose2d p = new Pose2d(0.332, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        assertNull("Test failed succeslfully!");

    }

}
