package org.team100.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command with an isDone() method, which makes composition in parallel groups
 * easier, because the command can be "done" without being "finished".
 */
public abstract class Done extends Command {
    public abstract boolean isDone();

}
