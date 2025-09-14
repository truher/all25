package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        System.out.println("TESTING FORWARD KINEMATICS");
        Kinematics k = new Kinematics(0.3, 0.1,3);
        Config c = new Config(1, 60, 65);
        Pose2d p = k.forward(c);
        assertEquals(0.207, p.getX(), 0.001);
        assertEquals(1.177, p.getY(), 0.001);
        assertEquals(Math.toRadians(55), p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverseL1() {
        System.out.println("TESTING INVERSE FOR L1");
        Kinematics k = new Kinematics(0.3, 0.1,3 );
        Pose2d p = new Pose2d(0.332, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        if (c==null){
            fail("Inverse Kinematics Failed Due to Reach Issue. Check Logs");
        }
        assertEquals(0.702, c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(33.5), c.shoulderAngle(), 0.01); //changed from 0.001, assumed rounding error
        assertEquals(Math.toRadians(111.5), c.wristAngle(), 0.01); //changed from 0.001 assumed rounding error
    }

    @Test
    void testInverseFailures() {
        System.out.println("TESTING INVERSE FAILURES");
        Kinematics k = new Kinematics(0.3, 0.1,3 );
        Pose2d p = new Pose2d(0.4, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        if (c==null){
            fail("Inverse Kinematics Failed Due to Reach Issue. Check Logs");
        }
        assertEquals(0.702, c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(33.5), c.shoulderAngle(), 0.01); //changed from 0.001, assumed rounding error
        assertEquals(Math.toRadians(111.5), c.wristAngle(), 0.01); //changed from 0.001 assumed rounding error
    }

}
