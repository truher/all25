package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;

/** Plays warnings using the CTRE music feature. */
public class Beeper {

    private final Machinery m_machinery;

    public Beeper(Machinery machinery) {
        m_machinery = machinery;
    }

    public Command play(double freq) {
        return parallel(
                m_machinery.m_mech.play(freq),
                m_machinery.m_manipulator.play(freq));
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
