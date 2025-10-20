package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;

/** Run pre- and post-match test sequences automatically. */
public class Tester {
    /**
     * TEST ALL MOVEMENTS
     * 
     * For pre- and post-match testing.
     * 
     * Enable "test" mode and press driver "a" and "b" together.
     * (in simulation this is buttons 1 and 2, or "z" and "x" on the keyboard)
     * 
     * DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
     *
     * THIS WILL MOVE THE ROBOT VERY FAST!
     *
     * DO NOT RUN with the wheels on the floor!
     *
     * DO NOT RUN without tiedown clamps.
     *
     * DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
     */
    public static Command prematch(Machinery m_machinery) {
        return sequence(
                print("*** WARNING! MOTION STARTS IN 4 SECONDS ***"),
                m_machinery.m_beeper.start(),
                print("ahead slow"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_drive.aheadSlow().withTimeout(1),
                print("rightward slow"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_drive.rightwardSlow().withTimeout(1),
                print("center intake"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_manipulator.centerIntake().withTimeout(1),
                print("L1"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_mech.homeToL1().withTimeout(1),
                print("home"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_mech.l1ToHome().withTimeout(1),
                print("L2"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_mech.homeToL2().withTimeout(1),
                print("home"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_mech.l2ToHome().withTimeout(1),
                print("done"),
                m_machinery.m_beeper.done())
                .withName("test all movements");
    }

}
