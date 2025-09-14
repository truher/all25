package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;
    private final double m_shoulderMaxHeight;

    public Kinematics(double armLength, double manipulatorLength, double shoulderMaxHeight) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;
        m_shoulderMaxHeight=shoulderMaxHeight;
    }

    public Pose2d forward(Config config) {
        // forward kinematics
        System.out.println("\nRUNNING FORWARD KINEMATICS: \n" );
        System.out.println("PRESETS:" );

        System.out.println("Arm Length: " + m_armLength);
        System.out.println("Manipulator Length: " + m_manipulatorLength);
        System.out.println("Shoulder Height: " + config.shoulderHeight());
        System.out.println("Shoulder Angle: " + config.shoulderAngle());
        System.out.println("Elbow Angle: " + config.wristAngle());

        double armXcomp  = (m_armLength * Math.cos(Math.toRadians(config.shoulderAngle())));
        double manipulatorXcomp = (m_manipulatorLength * Math.sin(Math.toRadians(config.wristAngle()-(180-90-config.shoulderAngle()))));
        double xComp = (armXcomp)+ (manipulatorXcomp);


        double armYcomp = (m_armLength * Math.sin(Math.toRadians(config.shoulderAngle())));
        double manipulatorYcomp = (m_manipulatorLength * Math.cos(Math.toRadians(config.wristAngle()-(180-90-config.shoulderAngle()))));
        double yComp =  (armYcomp) - (manipulatorYcomp) + config.shoulderHeight(); //subtraction becuse the manipulator is always pointing down?

        double manipulatorRotation = Math.toRadians(180-90-(config.wristAngle()-(180-90-config.shoulderAngle())));

        System.out.println("\nOUTPUTS:");
        System.out.println("armXcomp: " + armXcomp);
        System.out.println("armYcomp: " + armYcomp);
        System.out.println("manipulatorXcomp: " + manipulatorXcomp);
        System.out.println("manipulatorYcomp: " + manipulatorYcomp);
        System.out.println("X Point: " + (xComp));
        System.out.println("Y Point: " + yComp);
        System.out.println("Manipulator Rotation: " + (manipulatorRotation));

        return new Pose2d(xComp, yComp, new Rotation2d(manipulatorRotation));
    }

    public Config inverse(Pose2d pose) {
        // inverse kinematics

        System.out.println("\nRUNNING INVERSE KINEMATICS:\n" );

        System.out.println("PRESETS:" );
        System.out.println("Arm Length: " + m_armLength);
        System.out.println("Manipulator Length: " + m_manipulatorLength);
        System.out.println("Goal X: " + pose.getX());
        System.out.println("Goal Y: " + pose.getY());
        System.out.println("Goal Rotation: " + pose.getRotation().getDegrees());

        //0. Test if desired point is possible
        double xReach = (pose.getRotation().getCos()*m_manipulatorLength)+m_armLength;
        System.out.println("X reach: " +xReach);
        if (Math.abs(pose.getX())>xReach || pose.getY()>m_shoulderMaxHeight || pose.getY() <= 0) {

            if (pose.getX()>xReach){
                System.out.println("Desired X (" + pose.getX() + ") is greater than reach (" + xReach + ") and is not achievabl. Please test a different value.");
            }

            if (pose.getY()>m_shoulderMaxHeight || pose.getY() <= 0){
                System.out.println("Desired Y (" + pose.getY() + "is greater than max height (" + m_shoulderMaxHeight + ") or less than zero, and is not achievabl. Please test a different value.");
            }
            return null; //should cancel the program?

        } else {

            //1. find the elbow position from the manipulator length and the goal 2dpose. ('p' in my notes) - KYM (CORRECT - 9/13/2025)
            double elbowPointY = m_manipulatorLength*Math.sin(pose.getRotation().getRadians()) + pose.getY(); // vert side of triangle with hyp facing right, add key point height
            double wristPointX = pose.getX()-(m_manipulatorLength*Math.cos(pose.getRotation().getRadians())); //bottom of triangle with hyp facing right, subtract from key point

            
            //2. find the third leg of the triange formed by elbowPointX, hyp = armLength, and subtract that from the height 
            //of elbowpointY to find the actual shoulder joint height - KYM
            double shoulderHeight = elbowPointY-(Math.sqrt((m_armLength*m_armLength)-(wristPointX*wristPointX)));


            //find last side of triangle formed by both arms, vertex's being shoulderPt, goalPt, and elbowPt
            double zLength = Math.sqrt((pose.getX()*(pose.getX()))+((pose.getY()-shoulderHeight)*(pose.getY()-shoulderHeight)));
            
            //law of cosines to find the angle INSIDE the triangle that corresponds to the shoulder joint. second part is angle from 0 degrees to angle of elevation. gnarly
            //this part needs to be edited for going negative angles with arm?
            double shoulderAngle = (Math.acos((m_armLength*m_armLength + zLength*zLength - m_manipulatorLength*m_manipulatorLength) / (2 * m_armLength * zLength)))+((Math.asin((pose.getY()-shoulderHeight)/zLength)));

            //first part is law of cosidnes but for the elbow instead. 
            //need to think about how this will work with negative angles?
            double wristAngle = ((Math.acos((m_manipulatorLength*m_manipulatorLength + m_armLength*m_armLength - zLength*zLength) / (2 * m_manipulatorLength * m_armLength))));
            
            System.out.println("\nOUTPUTS:");
            System.out.println("Wrist Point (X,Y): (" + wristPointX + "," + elbowPointY + ")");
            System.out.println("Z length: " + zLength);
            System.out.println("Shoulder angle: "+ Math.toDegrees(shoulderAngle));
            System.out.println("Shoulder Height: "+ shoulderHeight);
            System.out.println("Wrist angle: " + Math.toDegrees(wristAngle));
            
            System.out.println("\nDEBUG:");
            System.out.println("Make sure this isn't greater than 1: " + ((m_manipulatorLength*m_manipulatorLength) + (m_armLength*m_armLength) - (zLength*zLength)) / (2 * m_manipulatorLength * m_armLength));

            return new Config(shoulderHeight, shoulderAngle, wristAngle);
        }
    }

}