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
    
    
    double insideSqrt;
    public double armHeightComp(Translation2d wristPosition)
    {
        if((m_armLength * m_armLength - wristPosition.getY() * wristPosition.getY())<0){
                insideSqrt=0;
        } else {
                insideSqrt = (m_armLength * m_armLength) - (wristPosition.getY()*wristPosition.getY());
        }

        return Math.sqrt(insideSqrt);
    }
    
    public Config inverse(Pose2d pose) {

        Translation2d wristPosition = pose.getTranslation()
                .minus(new Translation2d(m_manipulatorLength, pose.getRotation()));

        double direction, wristAngle;

        Translation2d ArmTranslation = new Translation2d(
                (armHeightComp(wristPosition)),
                wristPosition.getY()); 

        Translation2d shoulderHeight;
        if(pose.getX()<0.25){ //if less than crossover line, add the arm height (arm points down)
                 shoulderHeight = wristPosition.plus(ArmTranslation); 

        } else { //if goal point over the cross over line, have the arm point up
                 shoulderHeight = wristPosition.minus(ArmTranslation);
        }        
                wristAngle = (pose.getRotation().minus(ArmTranslation.getAngle()).getRadians()); //going down
         //-KYM

        return new Config(
                shoulderHeight.getX(),
                ArmTranslation.getAngle().getRadians(),
                wristAngle);
    }
}

