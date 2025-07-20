package org.team100.five_bar.setups;

import org.team100.five_bar.subsystems.FiveBarBare;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.XboxController;

public class SetupBare implements Runnable {
    private final FiveBarBare m_fiveBar;

    public SetupBare() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        XboxController controller = new XboxController(0);
        m_fiveBar = new FiveBarBare(logger);
        m_fiveBar.setDefaultCommand(m_fiveBar.dutyCycle(
                controller::getLeftX, controller::getRightX));
    }

    @Override
    public void run() {
        //
    }

}
