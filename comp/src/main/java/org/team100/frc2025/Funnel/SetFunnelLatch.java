package org.team100.frc2025.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFunnelLatch extends Command {
    private final Funnel m_funnel;
    private final double m_value1;
    private final double m_value2;

    public SetFunnelLatch(Funnel funnel, double value1, double value2) {
        m_funnel = funnel;
        m_value1 = value1;
        m_value2 = value2;
        addRequirements(m_funnel);
    }

    @Override
    public void execute() {
        m_funnel.setLatch1(m_value1);
        m_funnel.setLatch2(m_value2);
    }
}
