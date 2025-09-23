
// Kinematics class provides forward and inverse kinematics calculations for a robotic arm system.
// It uses arm length, wrist length, and elevator height to compute the position and configuration of the end effector.
package org.team100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


/**
 * Kinematics class for a robotic arm with an elevator, arm, and wrist.
 * Provides methods for forward and inverse kinematics.
 */
public class Kinematics {
    // Length of the arm segment (meters)
    private final double m_armLength;
    // Length of the wrist segment (meters)
    private final double m_wristLength;
    // Height of the elevator base (meters)
    private final double m_elevatorHeight;
    public Kinematics(double armLength, double wristLength, double elevatorHeight) {
        m_armLength = armLength;
        m_wristLength = wristLength;
        m_elevatorHeight = elevatorHeight;
    }
    public Pose2d forward(Config config) {
    // Calculates the end effector position and orientation using forward kinematics.
    // 1. Compute the height of the pivot point (base + elevator + offset)
    double pivotHeight = m_elevatorHeight + config.pivotHeightM();

    // 2. Compute the X position of the wrist joint
    //    - First term: arm length projected onto X axis
    //    - Second term: wrist length projected onto X axis (relative to arm angle + wrist angle)
    double WristX = m_armLength * Math.cos(config.pivotAngleRad())
        + m_wristLength * Math.cos(config.pivotAngleRad() + config.wristAngleRad());

    // 3. Compute the Y position of the wrist joint
    //    - First term: pivot height + arm length projected onto Y axis
    //    - Second term: wrist length projected onto Y axis (relative to arm angle + wrist angle)
    double WristY = pivotHeight
        + m_armLength * Math.sin(config.pivotAngleRad())
        + m_wristLength * Math.sin(config.pivotAngleRad() + config.wristAngleRad());

    // 4. Compute the final X position of the end effector
    double FinalX = WristX + m_wristLength * Math.cos(config.pivotAngleRad() + config.wristAngleRad());

    // 5. Compute the final Y position of the end effector
    double FinalY = WristY + m_wristLength * Math.sin(config.pivotAngleRad() + config.wristAngleRad());

    // 6. Return the pose (position and orientation) of the end effector
    return new Pose2d(FinalX, FinalY, new Rotation2d(config.pivotAngleRad() + config.wristAngleRad()));
    }


    public Config inverse(Pose2d pose) {
        // Extract the target X and Y position from the pose
        double x = pose.getX();
        System.out.println("X " + x);
        double y = pose.getY();
        System.out.println("Y " + y);
        // Extract the target orientation (angle) from the pose
        double Angel = pose.getRotation().getRadians();
        System.out.println("Angle " + Angel);
        // Calculate the wrist position by subtracting the wrist length from the target
        double WristX = x - m_wristLength * Math.cos(Angel);
        double WristY = y - m_wristLength * Math.sin(Angel + Math.toRadians(90));
        System.out.println("WristX: " + WristX);
        System.out.println("WristY: " + WristY);
        // Calculate the grip position relative to the elevator height
        double GripX = WristX - m_wristLength * Math.cos(Angel);
        double GripY = WristY - m_elevatorHeight;
        System.out.println("GripX: " + GripX);
        System.out.println("GripY: " + GripY);
        // Compute the distance from the base to the grip
        double distance = Math.sqrt(GripX * GripX + GripY * GripY);
        // Compute the angle from the base to the target
        double pivotTOTArgetAngle = Math.atan2(GripY, GripX);
        System.out.println("Distance: " + distance);
        System.out.println("Pivot to target angle: " + pivotTOTArgetAngle);
        // Law of cosines to find the bend angle at the shoulder joint
        double cosbendAngle = (-(distance * distance) + m_armLength * m_armLength + m_elevatorHeight * m_elevatorHeight) / (2 * m_elevatorHeight * m_armLength);
        cosbendAngle = Math.max(-1, Math.min(1, cosbendAngle)); // Constrct to valid range
        System.out.println("Cosine bend angle: " + cosbendAngle);
        double bendAngle = Math.acos(cosbendAngle);
        //System.out.println("Bend angle: " + bendAngle);
        // Calculate the required pivot angle
        double secondPivotAngle = pivotTOTArgetAngle + bendAngle;
        System.out.println("Second pivot angle: " + secondPivotAngle);
        // Calculate the required wrist angle
        double wristAngle = Angel - secondPivotAngle;
        //System.out.println("Wrist angle: " + wristAngle);
        // The pivot height is the elevator height
        double pivotHeight = m_elevatorHeight;
        //System.out.println("Pivot height: " + pivotHeight);
        // Return the config needed to reach the pose
        return new Config(secondPivotAngle, wristAngle, pivotHeight);
    }

}