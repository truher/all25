package org.team100;

import java.lang.management.GarbageCollectorMXBean;

import edu.wpi.first.math.geometry.Pose2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;
    private final double m_elevatorMaxHeight;
    private final double m_shoulderHeight;
    private final double m_elbowPointX;
    private final double m_elbowPointY;
    private final double m_goalX;
    private final double m_goalY;
    private final double m_goalR;
    private final double m_elbowAngle;
    private final double m_shoulderAngle;
    private final double m_zLength;



    public Kinematics(double armLength, double manipulatorLength, double elevatorMaxHeight, double goalX, double goalY, double goalR) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;
        m_elevatorMaxHeight = elevatorMaxHeight;

        m_goalX = goalX;
        m_goalY = goalY;
        m_goalR = goalR;

        //1. find the elbow position from the manipulator length and the goal 2dpose. ('p' in my notes) - KYM
        m_elbowPointY = manipulatorLength*Math.sin(Math.toRadians(goalR)); // vert side of triangle with hyp facing right
        m_elbowPointX = manipulatorLength*Math.cos(Math.toRadians(goalR)); //bottom of triangle with hyp facing right

        //2. find the third leg of the triange formed by elbowPointX, hyp = armLength, and subtract that from the height 
        //of elbowpointY to find the actual shoulder joint height - KYM
        m_shoulderHeight = m_elbowPointY-(Math.sqrt((armLength*armLength)-(m_elbowPointX*m_elbowPointX)));


        //find last side of triangle formed by both arms, vertex's being shoulderPt, goalPt, and elbowPt
        m_zLength = Math.sqrt(((goalX-0)*(goalX-0))+((goalY-m_shoulderHeight)*(goalY-m_shoulderHeight)));
        
        //law of cosines to find the angle INSIDE the triangle that corresponds to the shoulder joint
        // needs more work because that angle isn't relative to the angle of the elevator (-90)
        m_shoulderAngle = Math.toDegrees(Math.acos((armLength*armLength + m_zLength*m_zLength - manipulatorLength*manipulatorLength) / (2 * armLength * m_zLength)));

        //law of cosidnes but for the elbow instead
        m_elbowAngle = Math.toDegrees(Math.acos((manipulatorLength*manipulatorLength + armLength*armLength - m_zLength*m_zLength) / (2 * manipulatorLength * armLength)));

    }

    public Pose2d forward(Config config) {
        // forward kinematics
        return new Pose2d();
    }

    public Config inverse(Pose2d pose) {
        // inverse kinematics
        return new Config(0, 0, 0);
    }

}