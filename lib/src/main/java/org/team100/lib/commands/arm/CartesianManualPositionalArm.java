package org.team100.lib.commands.arm;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ArmAnglesLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive the arm to the specified cartesian position.
 * 
 * There are several flaws in this implementation.
 */
public class CartesianManualPositionalArm extends Command implements Glassy  {

    private final ArmSubsystem m_arm;
    private final ArmKinematics m_kinematics;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;

    private final PIDController m_lowerController;
    private final PIDController m_upperController;

    // LOGGERS

    private final Translation2dLogger m_log_input;
    private final ArmAnglesLogger m_log_setpoint;
    private final ArmAnglesLogger m_log_measurement;
    private final Translation2dLogger m_log_cartesian_measurement;
    private final DoubleLogger m_log_output_u1;
    private final DoubleLogger m_log_output_u2;
    private final DoubleLogger m_log_error_e1;
    private final DoubleLogger m_log_error_e2;

    public CartesianManualPositionalArm(
            LoggerFactory parent,
            ArmSubsystem arm,
            ArmKinematics kinematics,
            DoubleSupplier x,
            DoubleSupplier y) {
        LoggerFactory child = parent.child(this);
        m_log_input = child.translation2dLogger(Level.TRACE, "input");
        m_log_setpoint = child.armAnglesLogger(Level.TRACE, "setpoint");
        m_log_measurement = child.armAnglesLogger(Level.TRACE, "measurement");
        m_log_cartesian_measurement = child.translation2dLogger(Level.TRACE, "cartesian_measurement");
        m_log_output_u1 = child.doubleLogger(Level.TRACE, "output/u1");
        m_log_output_u2 = child.doubleLogger(Level.TRACE, "output/u2");
        m_log_error_e1 = child.doubleLogger(Level.TRACE, "error/e1");
        m_log_error_e2 = child.doubleLogger(Level.TRACE, "error/e2");

        m_arm = arm;
        m_kinematics = kinematics;
        m_x = x;
        m_y = y;

        m_lowerController = new PIDController(1, 0, 0);
        m_upperController = new PIDController(1, 0, 0);
        m_lowerController.enableContinuousInput(-Math.PI, Math.PI);
        m_upperController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(arm);
    }

    /**
     * Use inverse kinematics to transform the manual input into joint space.
     * This uses an offset to keep the inputs away from the origin.
     */
    @Override
    public void execute() {
        Translation2d input = new Translation2d(
                0.6 * m_x.getAsDouble() + 0.7,
                0.6 * m_y.getAsDouble() + 0.7);

        ArmAngles setpoint = m_kinematics.inverse(input);
        if (setpoint == null) {
            Util.warn("Ignoring infeasible input");
            return;
        }

        Optional<ArmAngles> measurement = m_arm.getPosition();
        if (measurement.isEmpty())
            return;

        Translation2d cartesian_measurement = m_kinematics.forward(measurement.get());

        double u1 = MathUtil.clamp(
                m_lowerController.calculate(measurement.get().th1, setpoint.th1), -1, 1);
        double u2 = MathUtil.clamp(
                m_upperController.calculate(measurement.get().th2, setpoint.th2), -1, 1);

        m_arm.set(u1, u2);

        m_log_input.log(() -> input);
        m_log_setpoint.log(() -> setpoint);
        m_log_measurement.log(measurement::get);
        m_log_cartesian_measurement.log(() -> cartesian_measurement);
        m_log_output_u1.log(() -> u1);
        m_log_output_u2.log(() -> u2);
        m_log_error_e1.log(m_lowerController::getPositionError);
        m_log_error_e2.log(m_upperController::getPositionError);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.set(0, 0);
    }

}