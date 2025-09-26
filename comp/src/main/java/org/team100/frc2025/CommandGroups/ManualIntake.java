package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import org.team100.frc2025.CalgamesArm.Placeholder;
import org.team100.frc2025.grip.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualIntake {

    /**
     * At the same time, move the arm to the floor and spin the intake,
     * endlessly.
     */
    public Command floor(Placeholder placeholder, Manipulator manipulator) {
        return parallel(
                placeholder.pick(),
                manipulator.centerIntake());
    }

    /**
     * At the same time, move the arm to the station and spin the intake,
     * endlessly.
     */
    public Command station(Placeholder placeholder, Manipulator manipulator) {
        return parallel(
                placeholder.station(),
                manipulator.centerIntake());
    }
}
