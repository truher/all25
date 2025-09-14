package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;
    private double m_shoulderHeight;
    private double m_elbowPointX;
    private double m_elbowPointY;
    private double m_elbowAngle;
    private double m_shoulderAngle;
    private double m_zLength;

    public Kinematics(double armLength, double manipulatorLength) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;
    }

    public Pose2d forward(Config config) {
        // forward kinematics
        System.out.println("TESTING FORWARD KINEMATICS: \n" );
        System.out.println("PRESETS:" );

        System.out.println("Arm Length: " + m_armLength);
        System.out.println("Manipulator Length: " + m_manipulatorLength);
        System.out.println("Shoulder Height: " + config.m_shoulderHeight());
        System.out.println("Shoulder Angle: " + config.m_shoulderAngle());
        System.out.println("Elbow Angle: " + config.m_elbowAngle());

        double armXcomp  = (m_armLength * Math.cos(Math.toRadians(config.m_shoulderAngle())));
        double manipulatorXcomp = (m_manipulatorLength * Math.sin(Math.toRadians(config.m_elbowAngle()-(180-90-config.m_shoulderAngle()))));
        double xComp = (armXcomp)+ (manipulatorXcomp);


        double armYcomp = (m_armLength * Math.sin(Math.toRadians(config.m_shoulderAngle())));
        double manipulatorYcomp = (m_manipulatorLength * Math.cos(Math.toRadians(config.m_elbowAngle()-(180-90-config.m_shoulderAngle()))));
        double yComp =  (armYcomp) - (manipulatorYcomp) + config.m_shoulderHeight(); //subtraction becuse the manipulator is always pointing down?

        double manipulatorRotation = Math.toRadians(180-90-(config.m_elbowAngle()-(180-90-config.m_shoulderAngle())));

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

        System.out.println("\nTESTING INVERSE KINEMATICS:\n" );

        System.out.println("PRESETS:" );
        System.out.println("Arm Length: " + m_armLength);
        System.out.println("Manipulator Length: " + m_manipulatorLength);
        System.out.println("Goal X: " + pose.getX());
        System.out.println("Goal Y: " + pose.getY());
        System.out.println("Goal Rotation: " + pose.getRotation().getDegrees());

        //1. find the elbow position from the manipulator length and the goal 2dpose. ('p' in my notes) - KYM (CORRECT - 9/13/2025)
        m_elbowPointY = m_manipulatorLength*Math.sin(pose.getRotation().getRadians()) + pose.getY(); // vert side of triangle with hyp facing right, add key point height
        m_elbowPointX = pose.getX()-(m_manipulatorLength*Math.cos(pose.getRotation().getRadians())); //bottom of triangle with hyp facing right, subtract from key point

        
        //2. find the third leg of the triange formed by elbowPointX, hyp = armLength, and subtract that from the height 
        //of elbowpointY to find the actual shoulder joint height - KYM
        m_shoulderHeight = m_elbowPointY-(Math.sqrt((m_armLength*m_armLength)-(m_elbowPointX*m_elbowPointX)));


        //find last side of triangle formed by both arms, vertex's being shoulderPt, goalPt, and elbowPt
        m_zLength = Math.sqrt((pose.getX()*(pose.getX()))+((pose.getY()-m_shoulderHeight)*(pose.getY()-m_shoulderHeight)));
        
        //law of cosines to find the angle INSIDE the triangle that corresponds to the shoulder joint. second part is angle from 0 degrees to angle of elevation. gnarly
        //this part needs to be edited for going negative angles with arm?
        m_shoulderAngle = (Math.acos((m_armLength*m_armLength + m_zLength*m_zLength - m_manipulatorLength*m_manipulatorLength) / (2 * m_armLength * m_zLength)))+((Math.asin((pose.getY()-m_shoulderHeight)/m_zLength)));

        //first part is law of cosidnes but for the elbow instead. 
        //need to think about how this will work with negative angles?
        m_elbowAngle = ((Math.acos((m_manipulatorLength*m_manipulatorLength + m_armLength*m_armLength - m_zLength*m_zLength) / (2 * m_manipulatorLength * m_armLength))));
        
        System.out.println("\nOUTPUTS:");
        System.out.println("Elbow Point (X,Y): (" + m_elbowPointX + "," + m_elbowPointY + ")");
        System.out.println("Z length: " + m_zLength);
        System.out.println("Shoulder angle: "+ Math.toDegrees(m_shoulderAngle));
        System.out.println("Shoulder Height: "+ m_shoulderHeight);
        System.out.println("Elbow angle: " + Math.toDegrees(m_elbowAngle));
        
        System.out.println("DEBUG:");
        System.out.println("Make sure this isn't greater than 1: " + ((m_manipulatorLength*m_manipulatorLength) + (m_armLength*m_armLength) - (m_zLength*m_zLength)) / (2 * m_manipulatorLength * m_armLength));

         return new Config(m_shoulderHeight, m_shoulderAngle, m_elbowAngle);
    }

}