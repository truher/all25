package org.team100.frc2025.CommandGroups.Hesitant;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class PreScoreL4Hesitant extends SequentialCommandGroup100 {

    public PreScoreL4Hesitant(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "PreScoreL4Hesitant");
        addCommands(
                new SetWrist(wrist, 0.4, false),
                new ParallelDeadlineGroup100(m_logger, "up",
                        new SetElevator(m_logger, elevator, 45, false),
                        new SetWrist(wrist, 0.4, true)), // 45
                new ParallelDeadlineGroup100(m_logger, "out",
                        new SetWrist(wrist, 1.25, true),
                        new SetElevatorPerpetually(elevator, 45)));
    }
}
