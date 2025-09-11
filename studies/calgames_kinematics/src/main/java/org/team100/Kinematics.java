package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;
    private double m_shoulderHeight;
    private double m_elbowPointX;
    private double m_elbowPointY;
    private double m_goalX;
    private double m_goalY;
    private double m_goalR;
    private double m_elbowAngle;
    private double m_shoulderAngle;
    private double m_zLength;
    
    //to do:
    //no consideration for the arm going down not up
    //no unit tests lmao
    //


    public Kinematics(double armLength, double manipulatorLength, double goalX, double goalY, double goalR) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;

    }

    public Pose2d forward(Config config) {
        // forward kinematics
        
        double xComp = m_armLength * Math.cos(m_shoulderAngle) + m_manipulatorLength * Math.cos(m_elbowAngle);
        double yComp =  m_armLength * Math.sin(m_shoulderAngle) + m_manipulatorLength * Math.sin(m_elbowAngle)+m_shoulderHeight;
        // m_goalX-xComp; the code you'd use to check if u got the right x,y (it should equal zero)
        // m_goalY-yComp;
        return new Pose2d(xComp, yComp, new Rotation2d(m_shoulderAngle+m_elbowAngle));
    }

    public Config inverse(Pose2d pose) {
        // inverse kinematics

        //the +90 is because the reef angle's will be opposite our angles right?
        m_goalR += 90;

        //1. find the elbow position from the manipulator length and the goal 2dpose. ('p' in my notes) - KYM
        m_elbowPointY = m_manipulatorLength*Math.sin(Math.toRadians(m_goalR)); // vert side of triangle with hyp facing right
        m_elbowPointX = m_manipulatorLength*Math.cos(Math.toRadians(m_goalR)); //bottom of triangle with hyp facing right

        //2. find the third leg of the triange formed by elbowPointX, hyp = armLength, and subtract that from the height 
        //of elbowpointY to find the actual shoulder joint height - KYM
        m_shoulderHeight = m_elbowPointY-(Math.sqrt((m_armLength*m_armLength)-(m_elbowPointX*m_elbowPointX)));


        //find last side of triangle formed by both arms, vertex's being shoulderPt, goalPt, and elbowPt
        m_zLength = Math.sqrt(((m_goalX-0)*(m_goalX-0))+((m_goalY-m_shoulderHeight)*(m_goalY-m_shoulderHeight)));
        
        //law of cosines to find the angle INSIDE the triangle that corresponds to the shoulder joint. second part is angle from 0 degrees to angle of elevation. gnarly
        //this part needs to be edited for going negative angles with arm?
        m_shoulderAngle = (Math.acos((m_armLength*m_armLength + m_zLength*m_zLength - m_manipulatorLength*m_manipulatorLength) / (2 * m_armLength * m_zLength)))+((Math.asin((m_goalY-m_shoulderHeight)/m_zLength)));

        //first part is law of cosidnes but for the elbow instead. 
        //need to think about how this will work with negative angles?
        m_elbowAngle = ((Math.acos((m_manipulatorLength*m_manipulatorLength + m_armLength*m_armLength - m_zLength*m_zLength) / (2 * m_manipulatorLength * m_armLength))));

         return new Config(m_shoulderHeight, m_shoulderAngle, m_elbowAngle);
    }

}