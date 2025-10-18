package org.team100.lib.config;

import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {

    private static final NamedChooser<Command> m_chooser = new NamedChooser<>("Auton Command");

    public AutonChooser() {
        m_chooser.setDefaultOption("NONE", Commands.print("*** No Auton Selected."));
        SmartDashboard.putData(m_chooser);
    }

    public void addAsDefault(String name, Command cmd) {
        m_chooser.setDefaultOption(name, cmd);
    }

    public void add(String name, Command command) {
        m_chooser.addOption(name, command);
    }

    public Command get() {
        return m_chooser.getSelected();
    }
}
