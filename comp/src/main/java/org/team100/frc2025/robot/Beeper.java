package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;

/** Plays warnings using the CTRE music feature. */
public class Beeper {

    Robot robot;

    public Beeper(Robot robot) {
        this.robot = robot;
    }

    public Command play(double freq) {
        return parallel(
                robot.m_mech.play(freq),
                robot.m_manipulator.play(freq));
    }

    /** Three beeps and one long beep. */
    public Command startingBeeps() {
        return sequence(
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(1760).withTimeout(1.0),
                play(0).withTimeout(0.1));
    }

}
