package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.lynxmotion_arm.AnalyticLynxArmKinematics;
import org.team100.lib.motion.lynxmotion_arm.LynxArmConfig;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics;
import org.team100.lib.motion.lynxmotion_arm.LynxArmPose;
import org.team100.lib.motion.lynxmotion_arm.NumericLynxArmKinematics;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmTest {
    private static final boolean DEBUG = true;

    @Test
    void testTwist() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new AnalyticLynxArmKinematics(0.07, 0.146, 0.187, 0.111);

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d start = new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(start);
            Pose3d end = new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            if (DEBUG) {
                System.out.printf("start %s\n", Util.poseStr(start));
                System.out.printf("end %s\n", Util.poseStr(end));
            }
            for (double s = 0; s <= 1; s += 0.2) {
                Pose3d lerp = start.interpolate(end, s);
                m_arm.setPosition(lerp);
                // wrist should be pointing down the whole time
                LynxArmPose p = m_arm.getPosition();
                if (DEBUG) {
                    System.out.printf("s %f p1 %s\n", s, Util.poseStr(p.p1()));
                    System.out.printf("s %f p2 %s\n", s, Util.poseStr(p.p2()));
                    System.out.printf("s %f p3 %s\n", s, Util.poseStr(p.p3()));
                    System.out.printf("s %f p4 %s\n", s, Util.poseStr(p.p4()));
                    System.out.printf("s %f p5 %s\n", s, Util.poseStr(p.p5()));
                }
            }
        }
    }

    @Test
    void testRoundTrip() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new AnalyticLynxArmKinematics(0.07, 0.146, 0.187, 0.111);

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.15, 0.1, 0),
                    new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", Util.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTripb() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new NumericLynxArmKinematics();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.15, 0.1, 0),
                    new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", Util.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTrip2() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new AnalyticLynxArmKinematics(0.07, 0.146,
                0.187, 0.111);

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.165326, 0.056973, 0.085628),
                    new Rotation3d(0.172187, 0.923138, -0.154456));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", Util.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTrip2b() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new NumericLynxArmKinematics();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            // there's something wrong with this setpoint.
            // Pose3d setpoint = new Pose3d(
            // new Translation3d(0.165326, 0.056973, 0.085628),
            // new Rotation3d(0.172187, 0.923138, -0.154456));
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.2, 0.0, 0.2),
                    new Rotation3d(0.0, 0.0, 0.0));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", Util.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig.str());
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig.str());
            assertEquals(measuredConfig.swing().getAsDouble(), commandedConfig.swing().getAsDouble(), 0.001);
            assertEquals(measuredConfig.boom(), commandedConfig.boom(), 0.001);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
            assertEquals(measuredConfig.wrist(), commandedConfig.wrist(), 0.001);
            assertEquals(measuredConfig.twist().getAsDouble(), commandedConfig.twist().getAsDouble(), 0.001);
        }
    }
}
