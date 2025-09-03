
package org.team100.frc2025.Funnel;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.frc2025.Climber.Climber;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;

public class ReleaseFunnel {
    public static Command get(LoggerFactory logger, Funnel funnel, Climber climber) {
        return sequence(
                funnel.setLatch(180, 0).withTimeout(1),
                climber.setPosition(2.51));
    }
}
