package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForward() {
        Kinematics k = new Kinematics(0.3, 0.1);
        Config c = new Config(1, Math.PI/2, Math.PI);
        Pose2d p = k.forward(c);
        assertEquals(1, p.getX(), 0.001);
        assertEquals(.4, p.getY(), 0.001);
        assertEquals(-Math.PI/2, p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverse() {
        Kinematics k = new Kinematics(0.3, 0.1);
        Pose2d p = new Pose2d(1, .4, new Rotation2d(-Math.PI/2));
        Config c = k.inverse(p);
        assertEquals(1, c.pivotHeightM(), 0.001);
        assertEquals(Math.PI/2, c.pivotAngleRad(), 0.001);
        assertEquals(Math.PI, c.wristAngleRad(), 0.001);
    }

    @Test
    void testIsPossible() {
        Kinematics k = new Kinematics(.3, 0.1);
        Pose2d p = new Pose2d(1, .4, new Rotation2d(-Math.PI/2));
        boolean possible = k.isPosePossible(p, 0, 0);
        if (possible){
            System.out.println("works");
        }
        else{
            System.out.println("doesn't work");
        }
    }
}
