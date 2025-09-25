package org.team100.frc2025.CalgamesArm;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.Viz;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Setup for cartesian control bindings, using inverse kinematics.
 */
public class CartesianSetup implements Runnable {
    private static final double CONTROL_SCALE = 0.1;
    // private final Mech m_mech;
    private final RealMech m_mech;
    private final Viz m_viz;

    public CartesianSetup() {
        Logging logging = Logging.instance();
        logging.setLevel(Level.TRACE);
        LoggerFactory logger = logging.rootLogger;
        // m_mech = Mech.make2025(logger);
        m_mech = RealMech.make2025(logger);
        m_viz = new Viz(m_mech);
        XboxController controller = new XboxController(0);
        m_mech.setDefaultCommand(m_mech.cartesian(
                // you'll want to redo these control mappings if you use a real controller
                () -> CONTROL_SCALE * controller.getRawAxis(0), // "a" and "d" in the sim
                () -> CONTROL_SCALE * controller.getRawAxis(1), // "w" and "s" in the sim
                () -> CONTROL_SCALE * controller.getRawAxis(2))); // "e" and "r" in the sim

    }

    @Override
    public void run() {
        m_mech.periodic();
        m_viz.periodic();
    }
}
