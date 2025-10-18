package org.team100.frc2025;

import org.team100.frc2025.robot.Robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
