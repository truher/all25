package frc.robot;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Simple arm visualization as an observer. */
public class Viz implements Runnable {
    private final DoubleSupplier m_observationRad;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_arm;

    public Viz(DoubleSupplier observationRad) {
        m_observationRad = observationRad;
        m_view = new Mechanism2d(100, 100);
        m_view.setBackgroundColor(new Color8Bit(Color.kLightCyan));
        m_root = m_view.getRoot("root", 50, 25);
        m_arm = new MechanismLigament2d("strut", 40, 0);
        m_arm.setColor(new Color8Bit(Color.kOrange));
        m_root.append(m_arm);
        SmartDashboard.putData("viz", m_view);
    }

    @Override
    public void run() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            m_arm.setAngle(Math.toDegrees(m_observationRad.getAsDouble()));
        }
    }

}
