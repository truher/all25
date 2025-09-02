package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckWristDanger;
import org.team100.frc2025.Wrist.IntakeAlgaeGrip;
import org.team100.frc2025.Wrist.SetWristPerpetually;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class GrabAlgaeL2Dumb extends SequentialCommandGroup100 {

    public GrabAlgaeL2Dumb(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            AlgaeGrip grip) {
        super(logger, "GrabAlgaeL2Dumb");
        addCommands(
                new CheckWristDanger(wrist),
                new ParallelDeadlineGroup100(m_logger, "grabL2",
                        new SetElevatorPerpetually(elevator, 23),
                        new SetWristPerpetually(wrist, 3.7),
                        new IntakeAlgaeGrip(grip)));
    }
}
