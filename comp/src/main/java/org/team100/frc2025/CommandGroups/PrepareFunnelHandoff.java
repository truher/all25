package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class PrepareFunnelHandoff extends SequentialCommandGroup100 {

    public PrepareFunnelHandoff(LoggerFactory logger, Wrist2 wrist, Elevator elevator) {
        super(logger, "PrepareFunnelHandoff");

        addCommands(
                new ParallelCommandGroup100(m_logger, "prep",
                        new SetWristHandoff(wrist, 0.1),
                        new SetElevatorFunnelHandoff(elevator, 0.1)));
    }
}
