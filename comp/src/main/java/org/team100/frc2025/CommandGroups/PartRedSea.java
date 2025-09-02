package org.team100.frc2025.CommandGroups;

import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevatorPerpetually;
import org.team100.frc2025.Wrist.SetWristHandoff;
import org.team100.frc2025.Wrist.SetWristPerpetually;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.framework.ParallelCommandGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;

public class PartRedSea extends SequentialCommandGroup100 {
    public PartRedSea(LoggerFactory logger, Wrist2 wrist, Elevator elevator, Climber climber) {
        super(logger, "PartRedSea");
        addCommands(
                new SetWristHandoff(wrist, 0.1),
                wrist.setDuty(-0.11).withTimeout(2),
                climber.setPosition(-3).until(() -> climber.atGoal()).withTimeout(4),
                new ParallelCommandGroup100(m_logger, "elevate",
                        new SetElevatorPerpetually(elevator, 10),
                        new SetWristPerpetually(wrist, 0.5)));
    }
}
