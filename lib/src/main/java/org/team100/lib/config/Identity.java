package org.team100.lib.config;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * Represents a specific RoboRIO, as a key for configurations.
 * 
 * The serial numbers here can be found on the label on the back: add a leading zero.
 * 
 * Please keep the ID strings in lexical order.
 * 
 * Note that the ID string may change when you reflash the RoboRIO.
 */

public enum Identity {
    TEST_BOARD_B0("030628b0"),
    ROOKIE_BOT("03063c8d"),
    FRC_100_ea4("0306cea4"),
    TEST_BOARD_6B("030d286b"),
    DEMO_BOT("03126d76"),
    TEAM100_2018("0313baf3"),
    BETA_BOT("0315db43"),
    SQUAREBOT("031e31e3"),
    SWERVE_TWO("0317f285"),

    COMP_BOT("03238232"),
    SWERVE_ONE("032363AC"),

    SYSTEMCORE("35af8fca4c12ad8e"), // new controller for 2027

    DISABLED("disabled"), // for mechanisms which don't exist
    BLANK(""), // e.g. test default or simulation
    UNKNOWN(null);
    // FRC_100_ea4("03238232"),
    // COMP_BOT("0306cea4");

    private static final Map<String, Identity> identities = new HashMap<>();

    static {
        for (Identity i : Identity.values()) {
            identities.put(i.m_serialNumber, i);
        }
    }

    public static final Identity instance = get();
    // for testing
    // public static final Identity instance = BETA_BOT;

    private final String m_serialNumber;

    private Identity(String serialNumber) {
        m_serialNumber = serialNumber;
    }

    private static Identity get() {
        String serialNumber = "";
        if (RobotBase.isReal()) {
            // Calling getSerialNumber in a vscode unit test
            // SEGVs because it does the wrong
            // thing with JNIs, so don't do that.
            serialNumber = RobotController.getSerialNumber();
            if (serialNumber.isBlank()) {
                // try again
                try (BufferedReader reader = new BufferedReader(new FileReader("/proc/cpuinfo"))) {
                    String line;
                    while ((line = reader.readLine()) != null) {
                        if (line.startsWith("Serial")) {
                            String[] parts = line.split(":");
                            if (parts.length > 1) {
                                serialNumber = parts[1].trim();
                                break;
                            }
                        }
                    }
                } catch (IOException e) {
                    //
                }
            }
        } else {
            serialNumber = "";
        }
        SmartDashboard.putString("serialNumber", serialNumber);
        if (identities.containsKey(serialNumber)) {
            Identity identity = identities.get(serialNumber);
            Util.printf("Identity: %s\n", identity);
            SmartDashboard.putString("identity", identity.name());
            return identity;
        }
        SmartDashboard.putString("identity", "UNKNOWN");
        Util.println("Identity: UNKNOWN");
        return UNKNOWN;
    }
}
