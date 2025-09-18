package org.team100;

import edu.wpi.first.math.geometry.Pose2d;

public class Kinematics {
    private final double m_armLength;
    private final double m_wristLength;

    public Kinematics(double armLength, double wristLength) {
        m_armLength = armLength;
        m_wristLength = wristLength;
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