// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025.Wrist;

import org.team100.frc2025.Funnel.Funnel;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandoffCommand extends ParallelCommandGroup {
  /** Creates a new HandoffCommand. */
  public HandoffCommand(Funnel funnel, Wrist2 wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new SetFunnelHandoff(wrist),
        // new Handoff(funnel, wrist)
    );
  }
}
