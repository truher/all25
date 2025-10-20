package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;

/** Plays warnings using the CTRE music feature. */
public class Beeper {
    private static final double BPM = 120;
    private static final double A5 = 880;
    private static final double A6 = 1670;

    private final Machinery m_machinery;

    public Beeper(Machinery machinery) {
        m_machinery = machinery;
    }

    public Command play(double freq) {
        return parallel(
                m_machinery.m_mech.play(freq),
                m_machinery.m_manipulator.play(freq),
                m_machinery.m_drive.play(freq));
    }

    /**
     * Three beeps and one long beep, approximately
     * 
     * | q qr q qr | q qr h |
     * 
     * at Allegro tempo.
     */
    public Command startingBeeps() {
        return sequence(
                quarterNote(A5),
                quarterRest(),
                quarterNote(A5),
                quarterRest(),
                quarterNote(A5),
                quarterRest(),
                halfNote(A6),
                halfRest());
    }

    public Command intermediateBeep() {
        return sequence(
                quarterNote(A5),
                quarterRest());
    }

    public Command quarterNote(double freq) {
        return play(freq).withTimeout(quarter());
    }

    public Command halfNote(double freq) {
        return play(freq).withTimeout(half());
    }

    public Command quarterRest() {
        return play(0).withTimeout(quarter());
    }

    public Command halfRest() {
        return play(0).withTimeout(half());
    }

    private double quarter() {
        return 60 / BPM;
    }

    private double half() {
        return 2 * 60 / BPM;
    }

}
