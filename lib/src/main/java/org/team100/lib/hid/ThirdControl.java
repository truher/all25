package org.team100.lib.hid;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.field.FieldConstants.ReefPoint;

/**
 * Represents a third control beyond the driver and operator, for example, knobs
 * for tuning, or a MIDI keyboard.
 */
public interface ThirdControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default boolean red1() {
        return false;
    }

    default boolean red2() {
        return false;
    }

    default boolean red3() {
        return false;
    }

    default boolean red4() {
        return false;
    }

    default boolean barge() {
        return false;
    }

    // Coral Scoring Levels

    default ScoringLevel level() {
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

    default boolean l1() {
        return false;
    }

    default boolean l2() {
        return false;
    }

    default boolean l3() {
        return false;
    }

    default boolean l4() {
        return false;
    }

    // Coral Scoring Reef Points

    default ReefPoint point() {
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

    default boolean a() {
        return false;
    }

    default boolean b() {
        return false;
    }

    default boolean c() {
        return false;
    }

    default boolean d() {
        return false;
    }

    default boolean e() {
        return false;
    }

    default boolean f() {
        return false;
    }

    default boolean g() {
        return false;
    }

    default boolean h() {
        return false;
    }

    default boolean i() {
        return false;
    }

    default boolean j() {
        return false;
    }

    default boolean k() {
        return false;
    }

    default boolean l() {
        return false;
    }

    // Algae locations between the poles

    default boolean algae() {
        return ab() || cd() || ef() || gh() || ij() || kl();
    }

    default ScoringLevel algaeLevel() {
        if (cd() || gh() || kl())
            return ScoringLevel.L2;
        if (ab() || ef() || ij())
            return ScoringLevel.L3;
        return ScoringLevel.NONE;
    }

    default boolean ab() {
        return false;
    }

    default boolean cd() {
        return false;
    }

    default boolean ef() {
        return false;
    }

    default boolean gh() {
        return false;
    }

    default boolean ij() {
        return false;
    }

    default boolean kl() {
        return false;
    }

}
