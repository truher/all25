package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import edu.wpi.first.wpilibj2.command.Command;

/** This exists to accumulate API for the arm. */
public class Placeholder {
    /** Move to central lower stowed position and hold it there forever. */
    public Command stow() {
        return print("stow");
    }

    /** True if the center of gravity is not too high. */
    public boolean isSafeToDrive() {
        return true;
    }

    /** Flop the arm into the station and hold it there forever. */
    public Command station() {
        // NOTE: be aware of position, don't try to do this if too close to the reef
        return print("station");
    }

    /** Move the arm to the coral floor pick and hold there forever. */
    public Command pick() {
        // NOTE: be aware of position, don't try to do this if too close to the reef or
        // the edge of the field.
        return print("pick");
    }

}
