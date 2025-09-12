package org.team100.frc2025.CommandGroups;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CheckFunnelDanger;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;

public class RunFunnelHandoff {
    /** sequence of handoff motions that doesn't finish. */
    public static Command get(
            LoggerFactory logger,
            Elevator elevator,
            Wrist2 wrist,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip) {
        return sequence(
                new CheckFunnelDanger(wrist, elevator),
                parallel(
                        wrist.set(0.1).until(wrist::atGoal),
                        elevator.setNoGravity(0.1).until(elevator::atGoal)),
                parallel(
                        funnel.agitate(),
                        tunnel.go(),
                        wrist.setDuty(-0.15)));
    }
}
