package org.team100.lib.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotDefault extends Command {
    private final Supplier<Double> m_input;
    private final PivotSubsystem m_pivot;

    public PivotDefault(Supplier<Double> input, PivotSubsystem pivot) {
        m_input = input;
        m_pivot = pivot;
        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        m_pivot.dutyCycle(MathUtil.applyDeadband(m_input.get(), 0.05) / 2);
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.stop();
    }
}
