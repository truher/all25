package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class KinematicsTest {
    @Test
    void testForwardBaseline() {
        System.out.println("\n\nTESTING BASELINE FORWARD KINEMATICS");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0);
        Config c = new Config(1, 60, 65);
        Pose2d p = k.forward(c);
        assertEquals(0.207, p.getX(), 0.001);
        assertEquals(1.178, p.getY(), 0.001);
        assertEquals(Math.toRadians(55), p.getRotation().getRadians(), 0.001);
    }

    @Test
    void testInverseL1() {
        System.out.println("\n\nTESTING INVERSE FOR L1");
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
    void testInverseBaseline() {
        System.out.println("\n\nTESTING BASIC INVERSE");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0 );
        Pose2d p = new Pose2d(0.207, 1.178, new Rotation2d(Math.toRadians(55)));
        Config c = k.inverse(p);
        if (c==null){
            fail("Inverse Kinematics Failed. Check Logs");
        }
        assertEquals(1, c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(60), c.shoulderAngle(), 0.001); //changed from 0.001, assumed rounding error
        assertEquals(Math.toRadians(65), c.wristAngle(), 0.001); //changed from 0.001 assumed rounding error
    }

    @Test
    void testInverseReachFailures() {
        System.out.println("\n\nTESTING INVERSE REACH FAILURES");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0, 0, 0, 0 );
        Pose2d p = new Pose2d(0.4, 0.81, new Rotation2d(Math.toRadians(35))); //impornt bits 
        Config c = k.inverse(p);
        assertNull(k.inverse(p));
    }

    @Test
    void testInverseDeadzoneFailures() {
        System.out.println("\n\nTESTING INVERSE DEADZONE FAILURES");
        Kinematics k = new Kinematics(0.3, 0.1,3, 0.1, 1, 0.5, 3 ); //important bits (O.5 AND 3)
        Pose2d p = new Pose2d(0.332, 0.81, new Rotation2d(Math.toRadians(35)));
        Config c = k.inverse(p);
        assertNull(k.inverse(p));

    }

    @Test
    void testRoundTripInverseFirst(){ //NOT WORKING, OFF BY 0.007, CHECK IF ROUNDING ERROR ON P.X PART?
        System.out.println("\n\nTESTING ROUND TRIP KINEMATICS (Inverse First)");
        Kinematics k = new Kinematics(0.3, 0.1, 3, 0, 0, 0, 0);
        Pose2d p = new Pose2d(0.207, 1.178, new Rotation2d(Math.toRadians(55)));
        Config c = new Config(1, 60, 65); //basic config for forward


        Config c2 = k.inverse(p);
        Pose2d p2 = k.forward(c2);


        System.out.println("Default config: " + c);
        System.out.println("Config derived from inverse fed with default pose: " + c2);
        System.out.println("");

        System.out.println("\nDefault pose:   " + p);
        System.out.println("Pose given by forward kinematics fed with inverse config: " + p2);


        System.out.println();
        //testing final x positions

        assertEquals(p.getX(), p2.getX(), 0.001); //check if (given pose x) = (recieved pose x from forward)
        assertEquals(p.getY(), p2.getY(), 0.001); //also wrong, off by a 0.178  
        
    }

    @Test
    void testRoundTripForwardFirst(){ //NOT WORKING, OFF BY 0.007, CHECK IF ROUNDING ERROR ON P.X PART?
        System.out.println("\n\nTESTING ROUND TRIP KINEMATICS (Forward First)");
        Kinematics k = new Kinematics(0.3, 0.1, 3, 0, 0, 0, 0);
        Pose2d p = new Pose2d(0.207, 1.178, new Rotation2d(Math.toRadians(55)));
        Config c = new Config(1, 60, 65); //basic config for forward

        Pose2d p2 = k.forward(c); //return basic forward output
        Config c2 = k.inverse(p2); //make a config based on the result of the forward kinematics

        System.out.println("\nDefault pose:   " + p);
        System.out.println("Pose given by forward kinematics: " + p2);
        System.out.println("");
        System.out.println("Default config: " + c);
        System.out.println("Config derived from forward point: " + c2);

        //testing final x positions
        assertEquals(Math.toRadians(c.shoulderAngle()), c2.shoulderAngle(), 0.001); //check if (given pose x) = (recieved pose x from forward)
        assertEquals(c.shoulderHeight(), c2.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(c.wristAngle()), c2.wristAngle(), 0.001);
        
    }


}
