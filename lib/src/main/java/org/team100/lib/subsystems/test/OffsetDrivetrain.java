package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class OffsetDrivetrain implements VelocitySubsystemR3 {
    private final VelocitySubsystemR3 m_delegate;
    private final Translation2d m_offset;

    public OffsetDrivetrain(VelocitySubsystemR3 delegate, Translation2d offset) {
        m_delegate = delegate;
        m_offset = offset;
    }

    @Override
    public ModelR3 getState() {
        ModelR3 state = m_delegate.getState();
        Pose2d statePose = state.pose();
        Rotation2d rotation = statePose.getRotation();
        Pose2d pose = new Pose2d(
                statePose.getTranslation().minus(m_offset), rotation);
        GlobalVelocityR3 v = state.velocity();
        double vx = v.x() - v.theta() * m_offset.rotateBy(rotation).getY();
        double vy = v.y() + v.theta() * m_offset.rotateBy(rotation).getX();
        double vtheta = v.theta();
        GlobalVelocityR3 vv = new GlobalVelocityR3(vx, vy, vtheta);
        return new ModelR3(pose, vv);
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    @Override
    public void setVelocity(GlobalVelocityR3 setpoint) {
        Rotation2d rotation = m_delegate.getState().pose().getRotation();
        double vx = setpoint.x() + setpoint.theta() * m_offset.rotateBy(rotation).getY();
        double vy = setpoint.y() - setpoint.theta() * m_offset.rotateBy(rotation).getX();
        double vtheta = setpoint.theta();
        m_delegate.setVelocity(new GlobalVelocityR3(vx, vy, vtheta));
    }

}
