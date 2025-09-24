package org.team100.lib.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Kinematics coordinates are as follows:
 * 
 * x axis is pointing up
 * y axis is pointing to the left
 * 
 * shoulder height is along x
 * shoulder angle is relative to straight-up, positive counterclockwise
 * wrist angle is relative to the arm, positive counterclockwise
 * 
 * I chose these coordinates so that none of the angles ever transit the
 * pi/-pi boundary, which makes control simpler.
 */

public class ElevatorArmWristKinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;
    private final double m_armCrossoverHeight;
    public ElevatorArmWristKinematics(double armLength, double manipulatorLength) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;
        m_armCrossoverHeight = 0.5; //mostly arbitrary, should be lower for lower CG
    }

    public Pose2d forward(Config config) {

        double x = config.shoulderHeight()
                + m_armLength * Math.cos(config.shoulderAngle())
                + m_manipulatorLength * Math.cos(config.shoulderAngle() + config.wristAngle());
        double y = m_armLength * Math.sin(config.shoulderAngle())
                + m_manipulatorLength * Math.sin(config.shoulderAngle() + config.wristAngle());
        double r = config.shoulderAngle() + config.wristAngle();
        return new Pose2d(x, y, new Rotation2d(r));

    }
    
    
    public double armX(Translation2d wrist) {
        double d = m_armLength * m_armLength - wrist.getY() * wrist.getY();
        if (d < 0) {
            // arm horizontal
            return 0;
        }
        if (wrist.getX() < m_armCrossoverHeight) {
            // reach down
            return -1.0 * Math.sqrt(d);
        }
        // reach up
        return Math.sqrt(d);
    }
    
    public Config inverse(Pose2d pose) {

        Translation2d wrist = pose.getTranslation()
                .minus(new Translation2d(m_manipulatorLength, pose.getRotation()));

        double direction, wristAngle;

        Translation2d shoulderToWrist = new Translation2d(
                (armX(wrist)),
                wrist.getY()); 

        Translation2d shoulder = wrist.minus(shoulderToWrist);    
        
        wristAngle = (pose.getRotation().minus(shoulderToWrist.getAngle()).getRadians()); //going down
         //-KYM

        return new Config(
                shoulder.getX(),
                shoulderToWrist.getAngle().getRadians(),
                wristAngle);
    }
}

