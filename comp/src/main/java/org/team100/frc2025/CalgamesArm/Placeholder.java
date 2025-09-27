package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.util.function.Supplier;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.wpilibj2.command.Command;

/** This exists to accumulate API for the arm. */
public class Placeholder {
    /** Move to central lower stowed position and hold it there forever. */
    public Command stow() {
        return print("stow");
    }

    /** True if the center of gravity is not too high. */
    public boolean isSafeToDrive() {
        return false;
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

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        return print("algae reef pick");
    }

    /**
     * Move to the L2 scoring position and hold there forever
     */
    public Command prePlaceL2() {
        return print("pre place L2");
    }

    /**
     * True if the L2 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    public boolean atL2() {
        return false;
    }

    /**
     * Move to the L3 scoring position and hold there forever
     */
    public Command prePlaceL3() {
        return print("pre place L3");
    }

    /**
     * True if the L3 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    public boolean atL3() {
        return false;
    }

    /**
     * Move to the L4 scoring position and hold there forever
     */
    public Command prePlaceL4() {
        return print("pre place L4");
    }

    /**
     * True if the L3 trajectory is done and error is within tolerance.
     * This should be done by the trajectory follower, not actually by the
     * placeholder class.
     */
    public boolean atL4() {
        return false;
    }

}
