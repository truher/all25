package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_wristLength;

    public static double clamp(double min, double max, double value){
        return Math.max(min, Math.min(max, value));
    }


    public Kinematics(double armLength, double wristLength) {
        m_armLength = armLength;
        m_wristLength = wristLength;
    }

    public Pose2d forward(Config config) {
        // forward kinematics

        //finding the wrist angle relative to a vertical pole using a parallelogram
        double globalWristAngle = (Math.PI - config.pivotAngleRad()) + config.wristAngleRad() - Math.PI;

        //finding x offset based off of pivot
        double y = m_armLength * Math.cos(config.pivotAngleRad() - Math.PI / 2);
        //adding x offset based off of wrist
        y += m_wristLength * Math.cos(globalWristAngle - Math.PI / 2);

        //finding y offset based off of pivot
        double x = m_armLength * Math.sin(config.pivotAngleRad() - Math.PI/2);
        //adding y offset based off of wrist
        x += m_wristLength * Math.sin(globalWristAngle - Math.PI/2);
        //adding elevator height
        x += config.pivotHeightM();

        //Optimal Angle
        Rotation2d theta = new Rotation2d(-(Math.PI - globalWristAngle));
        
        return new Pose2d(x, y, theta);
    }

    //Checks if it's possible, not if it should go there
    public boolean isPosePossible(Pose2d pose, double maxElevatorHeight, double minElevatorHeight){
        //setting a and b
        //a is the distance the manip-arm joint is away from the pole laterally (using law of sines)
        double a = (m_wristLength * Math.sin(Math.PI - ( - pose.getRotation().getRadians()))) / Math.sin(Math.PI/2);
    
        //b is the distance the manip arm joint is away from the pole vertically (using law of sines) 
        double b = (m_wristLength * Math.sin(( - pose.getRotation().getRadians()) - Math.PI/2)) / (Math.sin(Math.PI/2));
        
        //setting return value, using different shapes to see if it's inside of the possible area
        boolean posePossible = true;

        //Top Arc Equation is 

        //Checking if the pose is too far away laterally (with manipulator at the right angle)
        if (Math.abs(pose.getY()) > Math.abs(Math.signum(pose.getY()) * m_armLength + a)){
            posePossible = false;
        }
        //Checking if the pose is too high and outside of the max arc of the arm.
        else if (pose.getX() > maxElevatorHeight + Math.sqrt(Math.pow(m_armLength, 2) - Math.pow((pose.getX() - maxElevatorHeight) + a,2)) - b){
            posePossible = false;
        }
        //Checking if the pose is too low and outside of the max arc of the arm.
        else if (pose.getX() < minElevatorHeight - Math.sqrt(Math.pow(m_armLength, 2) - Math.pow((pose.getX() - maxElevatorHeight) + a,2)) - b){
            posePossible = false;
        }
        else if (pose.getX() < 0){
            posePossible = false;
        }

        return posePossible;
    }

    public Config inverse(Pose2d pose) {
        //setting distanceAway
        double distanceAway = pose.getY();

        //setting optimalAngle
        double optimalAngle = - pose.getRotation().getRadians();
        
        //setting poleHeight
        double poleHeight = pose.getX();

        //setting a and b
        //a is the distance the manip-arm joint is away from the pole laterally (using law of sines)
        double a = (m_wristLength * Math.sin(Math.PI - optimalAngle)) / Math.sin(Math.PI/2);
    
        //b is the distance the manip arm joint is away from the pole vertically (using law of sines) 
        //it's not needed though, it was for the thought process
        //double b = (m_wristLength * Math.sin(optimalAngle - Math.PI/2)) / (Math.sin(Math.PI/2));

        
        // inverse kinematics
        //using law of sines and geometry proofs to find missing angle
        // setting the inside asin to 1 if it surpases it or -1 if it goes below because of float error        
        double armAngle = Math.asin(clamp(-1.0, 1.0, (distanceAway - a) / m_armLength));
      
        //using known relations to get the missing angle
        double manipAngle = armAngle + (Math.PI - optimalAngle);

        /*set the line from the end point to top of elevator line using l1 and l2 with law of cosines equal
        to the pythagorean theorem of the same line but with distanceAway and difference between pole height and elevator height;
        then solve for the elevator height*/
        //Simplified: setting two equations of one line equal to each other with only 1 missing variable and solving for it

        double elevatorHeight = poleHeight - Math.sqrt(Math.max(0.0, Math.pow(m_armLength,2) + Math.pow(m_wristLength,2) - 2 * m_armLength * m_wristLength * Math.cos(manipAngle) - Math.pow(distanceAway, 2)));

        return new Config(elevatorHeight, armAngle, manipAngle);
    }
}