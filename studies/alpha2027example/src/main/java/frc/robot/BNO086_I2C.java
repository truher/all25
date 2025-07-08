package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.I2C;

/**
 * See https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
 * 
 * This is the minimal thing, to try to get it to work at all.
 */
public class BNO086_I2C {
    private static final boolean DEBUG = false;
    private static final byte ADDR = (byte) 0x4b;
    // this is a remnant from the small arduino buffers
    private static final int I2C_BUFFER_LENGTH = 32;
    private static final byte CHANNEL_EXECUTABLE = (byte) 0x01;
    private static final byte CHANNEL_CONTROL = (byte) 0x02;
    private static final byte CHANNEL_REPORTS = (byte) 0x03;
    // gyro gets its own channel :-)
    private static final byte CHANNEL_GYRO = (byte) 0x05;

    private static final byte SHTP_REPORT_COMMAND_RESPONSE = (byte) 0xF1;
    private static final byte SHTP_REPORT_PRODUCT_ID_RESPONSE = (byte) 0xF8;
    private static final byte SHTP_REPORT_PRODUCT_ID_REQUEST = (byte) 0xF9;
    private static final byte SHTP_REPORT_BASE_TIMESTAMP = (byte) 0xFB;
    private static final byte SHTP_REPORT_SET_FEATURE_COMMAND = (byte) 0xFD;

    private static final byte SENSOR_REPORTID_ACCELEROMETER = (byte) 0x01;
    private static final byte SENSOR_REPORTID_GYROSCOPE = (byte) 0x02;
    private static final byte SENSOR_REPORTID_MAGNETIC_FIELD = (byte) 0x03;
    private static final byte SENSOR_REPORTID_LINEAR_ACCELERATION = (byte) 0x04;
    private static final byte SENSOR_REPORTID_ROTATION_VECTOR = (byte) 0x05;
    private static final byte SENSOR_REPORTID_GRAVITY = (byte) 0x06;
    private static final byte SENSOR_REPORTID_UNCALIBRATED_GYRO = (byte) 0x07;
    private static final byte SENSOR_REPORTID_GAME_ROTATION_VECTOR = (byte) 0x08;
    private static final byte SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = (byte) 0x09;
    private static final byte SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR = (byte) 0x2A;
    private static final byte SENSOR_REPORTID_TAP_DETECTOR = (byte) 0x10;
    private static final byte SENSOR_REPORTID_STEP_COUNTER = (byte) 0x11;
    private static final byte SENSOR_REPORTID_STABILITY_CLASSIFIER = (byte) 0x13;
    private static final byte SENSOR_REPORTID_RAW_ACCELEROMETER = (byte) 0x14;
    private static final byte SENSOR_REPORTID_RAW_GYROSCOPE = (byte) 0x15;
    private static final byte SENSOR_REPORTID_RAW_MAGNETOMETER = (byte) 0x16;
    private static final byte SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = (byte) 0x1E;
    private static final byte SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR = (byte) 0x28;
    private static final byte SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR = (byte) 0x29;

    private static final byte COMMAND_ME_CALIBRATE = (byte) 0x07;

    private static final int MAX_PACKET_SIZE = 128;

    private static final byte rotationVector_Q1 = 14;
    private static final byte accelerometer_Q1 = 8;

    private final I2C m_i2c;

    // Each packet has a header of 4 bytes
    private final byte[] shtpHeader = new byte[4];

    // This is the same buffer for writes and for reads.
    // TODO: maybe don't do that?
    private final byte[] shtpData = new byte[MAX_PACKET_SIZE];

    // There are six channels. Each channel has its own sequence number.
    private final byte[] sequenceNumber = { 0, 0, 0, 0, 0, 0 };

    private int rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
    private int rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
    private int rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
    private int rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
    private int rawMagX, rawMagY, rawMagZ, magAccuracy;
    private int rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
    private int rawFastGyroX, rawFastGyroY, rawFastGyroZ;
    private int gravityX, gravityY, gravityZ, gravityAccuracy;
    private byte tapDetector;
    private int stepCount;
    private int memsRawAccelX, memsRawAccelY, memsRawAccelZ; // Raw readings from MEMS sensor
    private int memsRawGyroX, memsRawGyroY, memsRawGyroZ; // Raw readings from MEMS sensor
    private int memsRawMagX, memsRawMagY, memsRawMagZ; // Raw readings from MEMS sensor

    private int timeStamp;
    // Byte R0 of ME Calibration Response
    private byte calibrationStatus;

    public BNO086_I2C() {
        m_i2c = new I2C(I2C.Port.kPort0, ADDR);
    }

    public boolean begin() {
        // Begin by resetting the IMU
        softReset();

        // Check communication with device
        shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
        shtpData[1] = 0; // Reserved
        sendPacket(CHANNEL_CONTROL, 2);

        // Now we wait for response
        if (receivePacket()) {
            if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                if (DEBUG) {
                    System.out.printf("Version Major: %02X\n", shtpData[2]);
                    System.out.printf("Version Minor: %02X\n", shtpData[3]);
                    System.out.printf("Part Number: %02X%02X%02X%02X\n",
                            shtpData[7], shtpData[6], shtpData[5], shtpData[4]);
                    System.out.printf("Build Number: %02X%02X%02X%02X\n",
                            shtpData[11], shtpData[10], shtpData[9], shtpData[8]);
                    System.out.printf("Version Patch: %02X%02X\n",
                            shtpData[13], shtpData[12]);
                }
                return true;
            }
        }
        return false;
    }

    public void enableAccelerometer() {
        setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, 50);
    }

    public void enableRotationVector() {
        setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, 50);
    }

    public int getTimeStamp() {
        return timeStamp;
    }

    public float getQuatI() {
        return qToFloat(rawQuatI, rotationVector_Q1);
    }

    public float getQuatJ() {
        return qToFloat(rawQuatJ, rotationVector_Q1);
    }

    public float getQuatK() {
        return qToFloat(rawQuatK, rotationVector_Q1);
    }

    public float getQuatReal() {
        return qToFloat(rawQuatReal, rotationVector_Q1);
    }

    /** Rotation vector */
    public Quaternion getQuaternion() {
        return new Quaternion(getQuatReal(), getQuatI(), getQuatJ(), getQuatK());
    }

    public float getAccelX() {
        return qToFloat(rawAccelX, accelerometer_Q1);
    }

    public float getAccelY() {
        return qToFloat(rawAccelY, accelerometer_Q1);
    }

    public float getAccelZ() {
        return qToFloat(rawAccelZ, accelerometer_Q1);
    }

    public Vector<N3> getAccel() {
        return VecBuilder.fill(getAccelX(), getAccelY(), getAccelZ());
    }

    /**
     * Given a register value and a Q point, convert to float
     * See https://en.wikipedia.org/wiki/Q_(number_format)
     */
    public float qToFloat(int fixedPointValue, byte qPoint) {
        return (float) fixedPointValue * (float) Math.pow(2, qPoint * -1);
    }

    /**
     * Updates the latest variables if possible
     * 
     * @return false if new readings are not available
     */
    public boolean dataAvailable() {
        return (getReadings() != 0);
    }

    public int getReadings() {
        if (receivePacket()) {
            // Check to see if this packet is a sensor reporting its data to us
            if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
                // This will update the rawAccelX, etc variables depending on which feature
                // report is found
                return parseInputReport();
            } else if (shtpHeader[2] == CHANNEL_CONTROL) {
                // This will update responses to commands, calibrationStatus, etc.
                return parseCommandReport();
            } else if (shtpHeader[2] == CHANNEL_GYRO) {
                // This will update the rawAccelX, etc variables depending on which feature
                // report is found
                return parseInputReport();
            }

        }
        return 0;
    }

    /**
     * start a report. no "specific config" here since it's only used for
     * classifiers which we don't use.
     */
    public void setFeatureCommand(int reportID, int timeBetweenReports) {
        long microsBetweenReports = (long) timeBetweenReports * 1000L;

        shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; // Set feature command. Reference page 55
        shtpData[1] = (byte) (reportID & 0xFF); // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
        shtpData[2] = 0; // Feature flags
        shtpData[3] = 0; // Change sensitivity (LSB)
        shtpData[4] = 0; // Change sensitivity (MSB)

        shtpData[5] = (byte) ((microsBetweenReports >> 0) & 0xFF); // Report interval (LSB) in us. 0x7A120 = 500ms
        shtpData[6] = (byte) ((microsBetweenReports >> 8) & 0xFF); // Report interval
        shtpData[7] = (byte) ((microsBetweenReports >> 16) & 0xFF); // Report interval
        shtpData[8] = (byte) ((microsBetweenReports >> 24) & 0xFF); // Report interval (MSB)

        shtpData[9] = 0; // Batch Interval (LSB)
        shtpData[10] = 0; // Batch Interval
        shtpData[11] = 0; // Batch Interval
        shtpData[12] = 0; // Batch Interval (MSB)

        shtpData[13] = 0; // Sensor-specific config (LSB)
        shtpData[14] = 0; // Sensor-specific config
        shtpData[15] = 0; // Sensor-specific config
        shtpData[16] = 0; // Sensor-specific config (MSB)

        // Transmit packet on channel 2, 17 bytes
        sendPacket(CHANNEL_CONTROL, 17);
    }

    /**
     * 
     * Pulls the data from the input report
     * The input reports vary in length so this function stores the various 16-bit
     * values as globals
     * 
     * Unit responds with packet that contains the following:
     * 
     * shtpHeader[0:3]: header
     * shtpData[0:4]: microsecond since reading was taken
     * shtpData[5 + 0]: Feature report ID
     * shtpData[5 + 1]: Sequence number (See 6.5.18.2)
     * shtpData[5 + 2]: Status
     * shtpData[3]: Delay
     * shtpData[4:5]: i/accel x/gyro x/etc
     * shtpData[6:7]: j/accel y/gyro y/etc
     * shtpData[8:9]: k/accel z/gyro z/etc
     * shtpData[10:11]: real/gyro temp/etc
     * shtpData[12:13]: Accuracy estimate
     */
    public int parseInputReport() {
        int dataLength = uint16(shtpHeader[0], shtpHeader[1]);
        dataLength &= ~(1 << 15);
        // Remove the header bytes from the data count
        dataLength -= 4;
        timeStamp = uint32(shtpData[1], shtpData[2], shtpData[3], shtpData[4]);

        // The gyro-integrated input reports are sent via the special gyro channel and
        // omit the usual ID, sequence, and status fields
        if (shtpHeader[2] == CHANNEL_GYRO) {
            rawQuatI = uint16(shtpData[0], shtpData[1]);
            rawQuatJ = uint16(shtpData[2], shtpData[3]);
            rawQuatK = uint16(shtpData[4], shtpData[5]);
            rawQuatReal = uint16(shtpData[6], shtpData[7]);
            rawFastGyroX = uint16(shtpData[8], shtpData[9]);
            rawFastGyroY = uint16(shtpData[10], shtpData[11]);
            rawFastGyroZ = uint16(shtpData[12], shtpData[13]);
            return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
        }
        // Get status bits
        byte status = (byte) (shtpData[5 + 2] & 0x03);
        int data1 = uint16(shtpData[5 + 4], shtpData[5 + 5]);
        int data2 = uint16(shtpData[5 + 6], shtpData[5 + 7]);
        int data3 = uint16(shtpData[5 + 8], shtpData[5 + 9]);
        int data4 = 0;
        int data5 = 0;
        int data6 = 0;

        if (dataLength - 5 > 9) {
            data4 = uint16(shtpData[5 + 10], shtpData[5 + 11]);
        }
        if (dataLength - 5 > 11) {
            data5 = uint16(shtpData[5 + 12], shtpData[5 + 13]);
        }
        if (dataLength - 5 > 13) {
            data6 = uint16(shtpData[5 + 14], shtpData[5 + 15]);
        }

        // Store these generic values to their proper global variable
        if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER) {
            accelAccuracy = status;
            rawAccelX = data1;
            rawAccelY = data2;
            rawAccelZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION) {
            accelLinAccuracy = status;
            rawLinAccelX = data1;
            rawLinAccelY = data2;
            rawLinAccelZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE) {
            gyroAccuracy = status;
            rawGyroX = data1;
            rawGyroY = data2;
            rawGyroZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_UNCALIBRATED_GYRO) {
            UncalibGyroAccuracy = status;
            rawUncalibGyroX = data1;
            rawUncalibGyroY = data2;
            rawUncalibGyroZ = data3;
            rawBiasX = data4;
            rawBiasY = data5;
            rawBiasZ = data6;
        } else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD) {
            magAccuracy = status;
            rawMagX = data1;
            rawMagY = data2;
            rawMagZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
                shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
                shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
                shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) {
            quatAccuracy = status;
            rawQuatI = data1;
            rawQuatJ = data2;
            rawQuatK = data3;
            rawQuatReal = data4;

            // Only available on rotation vector and ar/vr stabilized rotation vector,
            // not game rot vector and not ar/vr stabilized rotation vector
            rawQuatRadianAccuracy = data5;
        } else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR) {
            tapDetector = shtpData[5 + 4]; // Byte 4 only
        } else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER) {
            stepCount = data3; // Bytes 8/9
        } else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
            // ignore it
        } else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
            // ignore it

        } else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER) {
            memsRawAccelX = data1;
            memsRawAccelY = data2;
            memsRawAccelZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE) {
            memsRawGyroX = data1;
            memsRawGyroY = data2;
            memsRawGyroZ = data3;
        } else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER) {
            memsRawMagX = data1;
            memsRawMagY = data2;
            memsRawMagZ = data3;
        } else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE) {
            if (DEBUG) {
                System.out.println("command response");
            }
            // The BNO080 responds with this report to command requests. It's up to use to
            // remember which command we issued.
            byte command = shtpData[5 + 2]; // This is the Command byte of the response

            if (command == COMMAND_ME_CALIBRATE) {
                if (DEBUG) {
                    System.out.println("ME Cal report found!");
                }
                calibrationStatus = shtpData[5 + 5]; // R0 - Status (0 = success, non-zero = fail)
            }
        } else if (shtpData[5] == SENSOR_REPORTID_GRAVITY) {
            gravityAccuracy = status;
            gravityX = data1;
            gravityY = data2;
            gravityZ = data3;
        } else {
            // This sensor report ID is unhandled.
            // See reference manual to add additional feature reports as needed
            return 0;
        }

        // TODO additional feature reports may be strung together. Parse them all.
        return shtpData[5];

    }

    /**
     * Pulls the data from the command response report
     * 
     * Unit responds with packet that contains the following:
     * 
     * shtpHeader[0:3]: header
     * shtpData[0]: Report ID
     * shtpData[1]: Sequence number (See 6.5.18.2)
     * shtpData[2]: Command
     * shtpData[3]: Command Sequence Number
     * shtpData[4]: Response Sequence Number
     * shtpData[5 + 0]: R0
     * shtpData[5 + 1]: R1
     * shtpData[5 + 2]: R2
     * shtpData[5 + 3]: R3
     * shtpData[5 + 4]: R4
     * shtpData[5 + 5]: R5
     * shtpData[5 + 6]: R6
     * shtpData[5 + 7]: R7
     * shtpData[5 + 8]: R8
     */
    public int parseCommandReport() {
        if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
            byte command = shtpData[2];
            if (command == COMMAND_ME_CALIBRATE) {
                // R0 - Status (0 = success, non-zero = fail)
                calibrationStatus = shtpData[5 + 0];
            }
            return shtpData[0];
        } else {
            // ignore it
        }
        return 0;
    }

    /**
     * Send command to reset IC.
     * Read all advertisement packets from sensor.
     */
    public void softReset() {
        shtpData[0] = 0x01;
        sendPacket(CHANNEL_EXECUTABLE, 1);

        delay(50);
        while (receivePacket()) {
            // slurp the whole packet
        }

        delay(50);
        while (receivePacket()) {
            // slurp the whole packet
        }
    }

    /**
     * Compute and send the header and then some bytes of shtpData.
     * Note the sequence number.
     * 
     * @return false if no ACK
     */
    public boolean sendPacket(int channelNumber, int dataLength) {
        // Add four bytes for the header
        int packetLength = dataLength + 4;

        byte[] payload = new byte[packetLength];

        payload[0] = (byte) (packetLength & 0xFF);
        payload[1] = (byte) ((packetLength >> 8) & 0xFF);
        payload[2] = (byte) (channelNumber & 0xFF);
        payload[3] = (byte) ((sequenceNumber[channelNumber]++) & 0xFF);

        System.arraycopy(shtpData, 0, payload, 4, dataLength);

        boolean aborted = m_i2c.writeBulk(payload);
        return !aborted;
    }

    /**
     * Check to see if there is any new data available
     * Read the contents of the incoming packet into shtpHeader and shtpData.
     * 
     * @return false if no data available
     */
    public boolean receivePacket() {
        // Ask for four bytes to find out how much data we need to read
        byte[] payload = new byte[4];
        m_i2c.readOnly(payload, 4);

        byte packetLSB = payload[0];
        byte packetMSB = payload[1];
        byte channelNumber = payload[2];
        byte sequenceNumber = payload[3];

        shtpHeader[0] = packetLSB;
        shtpHeader[1] = packetMSB;
        shtpHeader[2] = channelNumber;
        shtpHeader[3] = sequenceNumber;

        int dataLength = uint16(packetLSB, packetMSB);
        // Clear the MSbit. This bit indicates if this package is a continuation of the
        // last. Ignore it for now.
        dataLength &= ~(1 << 15);

        if (dataLength == 0) {
            // Packet is empty
            return false;
        }
        // Remove the header bytes from the data count
        dataLength -= 4;

        getData(dataLength);

        return true;
    }

    /**
     * Sends multiple requests to sensor until all data bytes are received from
     * sensor.
     */
    public void getData(int bytesRemaining) {
        // Start at the beginning of shtpData array
        short dataSpot = 0;
        // Setup a series of chunked 32 byte reads
        while (bytesRemaining > 0) {
            int numberOfBytesToRead = bytesRemaining;
            if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4)) {
                numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);
            }
            byte[] payload = new byte[numberOfBytesToRead + 4];
            m_i2c.readOnly(payload, numberOfBytesToRead + 4);
            for (int i = 0; i < numberOfBytesToRead; ++i) {
                byte incoming = payload[i + 4];
                if (dataSpot < MAX_PACKET_SIZE) {
                    shtpData[dataSpot++] = incoming;
                } // ignore bytes after that (but keep requesting them)
            }
            bytesRemaining -= numberOfBytesToRead;
        }
    }

    public void delay(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public int uint16(byte lsb, byte msb) {
        return (Byte.toUnsignedInt(msb) << 8) | Byte.toUnsignedInt(lsb);
    }

    public int uint32(byte lsb, byte b2, byte b3, byte msb) {
        return (Byte.toUnsignedInt(msb) << 24)
                | (Byte.toUnsignedInt(b2) << 16)
                | (Byte.toUnsignedInt(b3) << 8)
                | Byte.toUnsignedInt(lsb);
    }
}
