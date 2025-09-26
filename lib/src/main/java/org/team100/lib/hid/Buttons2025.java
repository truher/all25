package org.team100.lib.hid;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.field.FieldConstants.ReefPoint;

import edu.wpi.first.wpilibj.GenericHID;

public class Buttons2025 implements ThirdControl {
    private final GenericHID m_controller;

    public Buttons2025() {
        m_controller = new GenericHID(2);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean red1() {
        return m_controller.getRawButton(20);
    }

    @Override
    public boolean red2() {
        return m_controller.getRawButton(19);
    }

    @Override
    public boolean red3() {
        return m_controller.getRawButton(18);
    }

    @Override
    public boolean red4() {
        return m_controller.getRawButton(17);
    }

    @Override
    public boolean barge() {
        return m_controller.getRawButton(22);
    }

    @Override
    public ScoringLevel level() {
        if (l1()) {
            return ScoringLevel.L1;
        } else if (l2()) {
            return ScoringLevel.L2;
        } else if (l3()) {
            return ScoringLevel.L3;
        } else if (l4()) {
            return ScoringLevel.L4;
        }

        return ScoringLevel.NONE;
    }

    @Override
    public boolean l1() {
        return m_controller.getRawButton(5);
    }

    @Override
    public boolean l2() {
        return m_controller.getRawButton(6);
    }

    @Override
    public boolean l3() {
        return m_controller.getRawButton(7);
    }

    @Override
    public boolean l4() {
        return m_controller.getRawButton(4);
    }

    @Override
    public ReefPoint point() {
        if (a())
            return ReefPoint.A;
        if (b())
            return ReefPoint.B;
        if (c())
            return ReefPoint.C;
        if (d())
            return ReefPoint.D;
        if (e())
            return ReefPoint.E;
        if (f())
            return ReefPoint.F;
        if (g())
            return ReefPoint.G;
        if (h())
            return ReefPoint.H;
        if (i())
            return ReefPoint.I;
        if (j())
            return ReefPoint.J;
        if (k())
            return ReefPoint.K;
        if (l())
            return ReefPoint.L;
        if (ab())
            return ReefPoint.AB;
        if (cd())
            return ReefPoint.CD;
        if (ef())
            return ReefPoint.EF;
        if (gh())
            return ReefPoint.GH;
        if (ij())
            return ReefPoint.IJ;
        if (kl())
            return ReefPoint.KL;
        return ReefPoint.NONE;
    }

    @Override
    public boolean a() {
        return m_controller.getRawButton(15);
    }

    @Override
    public boolean b() {
        return m_controller.getRawButton(21);
    }

    @Override
    public boolean c() {
        return m_controller.getRawButton(31);
    }

    @Override
    public boolean d() {
        return m_controller.getRawButton(30);
    }

    @Override
    public boolean e() {
        return m_controller.getRawButton(28);
    }

    @Override
    public boolean f() {
        return m_controller.getRawButton(27);
    }

    @Override
    public boolean g() {
        return m_controller.getRawButton(25);
    }

    @Override
    public boolean h() {
        return m_controller.getRawButton(23);
    }

    @Override
    public boolean i() {
        return m_controller.getRawButton(9);
    }

    @Override
    public boolean j() {
        return m_controller.getRawButton(10);
    }

    @Override
    public boolean k() {
        return m_controller.getRawButton(12);
    }

    @Override
    public boolean l() {
        return m_controller.getRawButton(13);
    }

    @Override
    public boolean ab() {
        return m_controller.getRawButton(1);
    }

    @Override
    public boolean cd() {
        return m_controller.getRawButton(2);
    }

    @Override
    public boolean ef() {
        return m_controller.getRawButton(29);
    }

    @Override
    public boolean gh() {
        return m_controller.getRawButton(26);
    }

    @Override
    public boolean ij() {
        return m_controller.getRawButton(11);
    }

    @Override
    public boolean kl() {
        return m_controller.getRawButton(14);
    }

}
