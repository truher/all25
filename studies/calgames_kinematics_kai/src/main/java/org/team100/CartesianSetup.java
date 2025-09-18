package org.team100;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Setup for cartesian control bindings, using inverse kinematics.
 */
public class CartesianSetup implements Runnable {
    private static final double CONTROL_SCALE = 0.1;
    private final Mech m_mech;
    private final Viz m_viz;

    public CartesianSetup() {
        m_mech = Mech.make2025();
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
        m_viz.periodic();
    }
}
