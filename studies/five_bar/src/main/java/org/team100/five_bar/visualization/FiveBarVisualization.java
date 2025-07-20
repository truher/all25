package org.team100.five_bar.visualization;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.motion.five_bar.JointPositions;
import org.team100.lib.motion.five_bar.Point;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Uses the Mechanism2d widget to visualize the five-bar.
 * 
 * The widget uses a sequence of links. The length and parent-relative angle of
 * each link can be adjusted.
 * 
 * The forward kinematics of the five-bar produces cartesian points, so we have
 * to extract the link-sequence angles and lengths from them.
 * 
 * The root link ("a5") is not rendered.
 */
public class FiveBarVisualization {
    private static final Color8Bit ORANGE = new Color8Bit(Color.kOrangeRed);
    private final Supplier<JointPositions> m_q;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_a1;
    private final MechanismLigament2d m_a2;
    private final MechanismLigament2d m_a3;
    private final MechanismLigament2d m_a4;

    public FiveBarVisualization(Supplier<JointPositions> q) {
        m_q = q;
        m_view = new Mechanism2d(100, 100);
        m_root = m_view.getRoot("root", 50, 50);
        m_a1 = new MechanismLigament2d("a1", 0, 0, 0, ORANGE);
        m_root.append(m_a1);
        m_a2 = new MechanismLigament2d("a2", 0, 0, 0, ORANGE);
        m_a1.append(m_a2);
        m_a3 = new MechanismLigament2d("a3", 0, 0, 0, ORANGE);
        m_a2.append(m_a3);
        m_a4 = new MechanismLigament2d("a4", 0, 0, 0, ORANGE);
        m_a3.append(m_a4);
        SmartDashboard.putData("View", m_view);
    }

    public void periodic() {
        JointPositions q = m_q.get();
        List<Point> p = links(q);
        m_a1.setLength(p.get(0).norm());
        m_a2.setLength(p.get(1).norm());
        m_a3.setLength(p.get(2).norm());
        m_a4.setLength(p.get(3).norm());
        m_a1.setLength(p.get(0).angle().orElse(0));
        m_a2.setLength(p.get(1).angle().orElse(0));
        m_a3.setLength(p.get(2).angle().orElse(0));
        m_a4.setLength(p.get(3).angle().orElse(0));
    }

    static List<Point> links(JointPositions q) {
        Point a1 = q.P2().minus(q.P1());
        Point a2 = q.P3().minus(q.P2());
        Point a3 = q.P4().minus(q.P3());
        Point a4 = q.P5().minus(q.P4());
        return List.of(
                a1,
                a2.rotateBy(-1.0 * a1.angle().orElse(0)),
                a3.rotateBy(-1.0 * a2.angle().orElse(0)),
                a4.rotateBy(-1.0 * a3.angle().orElse(0)));
    }

}
