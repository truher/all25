package org.team100.frc2025.shooter.indexer;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Indexer extends Subsystem {

    public void set(double value);

    public void stop();
} 