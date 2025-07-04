package frc.robot.bno086_usc;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.I2C;

/**
 * This is part of the USC RPL library, removing most of it.
 * 
 * I'm not confident about the low-level i2c part, but it's worth a try.
 */
public class BNO080 {
    /// i2c address of IMU (7 bits)
    private static final byte ADDR = (byte) 0x4a; // 0x4b; //??
    private static final int CHANNEL_EXECUTABLE = 1;
    private static final int CHANNEL_CONTROL = 2;
    private static final int CHANNEL_REPORTS = 3;
    private static final int CHANNEL_WAKE_REPORTS = 4;
    private static final int SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8;
    private static final int SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9;
    private static final int SHTP_REPORT_BASE_TIMESTAMP = 0xFB;
    private static final int SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD;
    private static final int SENSOR_REPORTID_TIMESTAMP_REBASE = 0xFA;
    private static final int SENSOR_REPORTID_GYROSCOPE_CALIBRATED = 0x02;
    private static final int SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08;
    // Q points for various sensor data elements
    private static final int GYRO_Q_POINT = 9;// for gyroscope data
    private static final int ROTATION_Q_POINT = 14;// for rotation data
    // Report IDs on the Executable channel
    // See Figure 1-27 in the BNO080 datasheet
    private static final int EXECUTABLE_REPORTID_RESET = 0x1;
    // sizes of various sensor data packet elements
    private static final int SIZEOF_BASE_TIMESTAMP = 5;
    private static final int SIZEOF_TIMESTAMP_REBASE = 5;
    private static final int SIZEOF_GYROSCOPE_CALIBRATED = 10;
    private static final int SIZEOF_GAME_ROTATION_VECTOR = 12;
    private static final int SHTP_HEADER_SIZE = 4;
    // Size of the largest individual packet we can receive.
    // Min value is set by the advertisement packet (272 bytes)
    // If you enable lots of sensor reports and get an error, you might need to
    // increase this.
    private static final int SHTP_RX_PACKET_SIZE = 272;
    // Size of largest packet that we need to transmit (not including header)
    private static final int SHTP_MAX_TX_PACKET_SIZE = 17;
    // scratch space buffers
    private byte[] txPacketBuffer = new byte[SHTP_HEADER_SIZE + SHTP_MAX_TX_PACKET_SIZE];
    // need a second header worth of extra scratch space to write the header of a
    // continued packet
    private byte[] rxPacketBuffer = new byte[SHTP_HEADER_SIZE + SHTP_RX_PACKET_SIZE + SHTP_HEADER_SIZE];
    /// Length of packet that was received into buffer. Does NOT include header
    /// bytes.
    private int rxPacketLength;
    /// stores whether a sensor has been updated since the last call to hasNewData()
    private boolean[] reportHasBeenUpdated = new boolean[0x09];

    /// List of all sensor reports that the IMU supports.
    enum Report {
        /**
         * (calibrated) gyroscope reading of the rotational speed of the IMU.
         * See BNO datasheet section 2.1.2
         */
        GYROSCOPE(SENSOR_REPORTID_GYROSCOPE_CALIBRATED),

        /**
         * Fused reading of the IMU's rotation in space. Unlike the regular rotation
         * vector, the Game Rotation Vector
         * is not referenced against the magnetic field and the "zero yaw" point is
         * arbitrary.
         * See BNO datasheet section 2.2.2
         */
        GAME_ROTATION(SENSOR_REPORTID_GAME_ROTATION_VECTOR);

        private final int value;

        private Report(int v) {
            this.value = v;
        }
    };

    /// Version info read from the IMU when it starts up
    public byte majorSoftwareVersion;
    public byte minorSoftwareVersion;
    public short patchSoftwareVersion;
    public int partNumber;
    public int buildNumber;

    /**
     * Readout from Calibrated Gyroscope report
     * Represents the angular velocities of the chip in rad/s in the X, Y, and Z
     * axes
     */
    public Vector<N3> gyroRotation;

    /**
     * Readout from the Game Rotation Vector report.
     * Represents the rotation of the IMU in radians. Unlike the regular rotation
     * vector, the Game Rotation Vector
     * is not referenced against the magnetic field and the "zero yaw" point is
     * arbitrary.
     */
    public Quaternion gameRotationVector;

    private final I2C m_i2c;

    public BNO080() {
        m_i2c = new I2C(I2C.Port.kPort0, ADDR);
    }

    /**
     * Resets and connects to the IMU. Verifies that it's connected, and reads out
     * its version
     * info into the class variables above.
     *
     * If this function is failing, it would be a good idea to turn on BNO_DEBUG in
     * the cpp file to get detailed output.
     *
     * Note: this function takes several hundred ms to execute, mainly due to
     * waiting for the BNO to boot.
     *
     * @return initialization success
     */

    public boolean begin() {

        // At system startup, the hub must send its full advertisement message (see SHTP
        // 5.2 and 5.3) to the
        // host. It must not send any other data until this step is complete.
        // We don't actually care what's in it, we're just using it as a signal to
        // indicate that the reset is complete.
        receivePacket();

        // now, after startup, the BNO will send an Unsolicited Initialize response
        // (SH-2 section 6.4.5.2), and an Executable Reset command
        if (!readPacket(CHANNEL_EXECUTABLE, EXECUTABLE_REPORTID_RESET)) {
            return false;
        }

        // Finally, we want to interrogate the device about its model and version.
        clearSendBuffer();
        txPacketBuffer[4] = (byte) SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
        txPacketBuffer[5] = 0; // Reserved
        sendPacket(CHANNEL_CONTROL, 2);

        readPacket(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE);

        if (rxPacketBuffer[4] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            majorSoftwareVersion = rxPacketBuffer[6];
            minorSoftwareVersion = rxPacketBuffer[7];
            patchSoftwareVersion = (short) ((rxPacketBuffer[17] << 8) | rxPacketBuffer[16]);
            partNumber = (rxPacketBuffer[11] << 24) | (rxPacketBuffer[10] << 16) | (rxPacketBuffer[9] << 8)
                    | rxPacketBuffer[8];
            buildNumber = (rxPacketBuffer[15] << 24) | (rxPacketBuffer[14] << 16) | (rxPacketBuffer[13] << 8)
                    | rxPacketBuffer[12];

        } else {
            return false;
        }

        // reports we use
        enableReport(Report.GAME_ROTATION, 50);
        enableReport(Report.GYROSCOPE, 50);

        return true;
    }

    /**
     * Checks if a specific report has gotten new data since the last call to this
     * function.
     * 
     * @param report The report to check.
     * @return Whether the report has received new data.
     */
    public boolean hasNewData(Report report) {
        byte reportNum = (byte) (report.value);
        boolean newData = reportHasBeenUpdated[reportNum];
        reportHasBeenUpdated[reportNum] = false; // clear flag
        return newData;
    }

    /**
     * Enable a data report from the IMU. Look at the comments above to see what the
     * reports do.
     * This function checks your polling period against the report's max speed in
     * the IMU's metadata,
     * and reports an error if you're trying to poll too fast.
     *
     * set polling period to zero to disable it
     * 
     * @param timeBetweenReports time in milliseconds between data updates.
     */
    private void enableReport(Report report, int timeBetweenReports) {
        setFeatureCommand((byte) (report.value), timeBetweenReports);
    }

    /**
     * Processes the packet currently stored in the buffer, and updates class
     * variables to reflect the data it contains
     */
    private void processPacket() {
        if (rxPacketBuffer[2] == CHANNEL_REPORTS || rxPacketBuffer[2] == CHANNEL_WAKE_REPORTS) {
            if (rxPacketBuffer[4] == SHTP_REPORT_BASE_TIMESTAMP) {
                // every sensor data report first contains a timestamp offset
                parseSensorDataPacket();
            }
        }
    }

    /**
     * Processes the sensor data packet currently stored in the buffer.
     * Only called from processPacket()
     */
    private void parseSensorDataPacket() {
        int currReportOffset = 0;

        // every sensor data report first contains a timestamp offset to show how long
        // it has been between when
        // the host interrupt was sent and when the packet was transmitted.
        // We don't use interrupts and don't care about times, so we can throw this out.
        currReportOffset += SIZEOF_BASE_TIMESTAMP;

        while (currReportOffset < rxPacketLength) {
            // lots of sensor reports use 3 16-bit numbers stored in bytes 4 through 9
            // we can save some time by parsing those out here.
            short data1 = (short) (rxPacketBuffer[currReportOffset + 9] << 8 | rxPacketBuffer[currReportOffset + 8]);
            short data2 = (short) (rxPacketBuffer[currReportOffset + 11] << 8 | rxPacketBuffer[currReportOffset + 10]);
            short data3 = (short) (rxPacketBuffer[currReportOffset + 13] << 8 | rxPacketBuffer[currReportOffset + 12]);

            byte reportNum = rxPacketBuffer[currReportOffset + 4];

            if (reportNum != SENSOR_REPORTID_TIMESTAMP_REBASE) {
                // set status from byte 2

                // set updated flag
                reportHasBeenUpdated[reportNum] = true;
            }

            int selector = (int) rxPacketBuffer[currReportOffset + 4];
            switch (selector) {
                case SENSOR_REPORTID_TIMESTAMP_REBASE:
                    currReportOffset += SIZEOF_TIMESTAMP_REBASE;
                    break;

                case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:

                    gyroRotation = VecBuilder.fill(
                            qToFloat(data1, GYRO_Q_POINT),
                            qToFloat(data2, GYRO_Q_POINT),
                            qToFloat(data3, GYRO_Q_POINT));

                    currReportOffset += SIZEOF_GYROSCOPE_CALIBRATED;
                    break;

                case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {
                    short realPartQ = (short) (rxPacketBuffer[currReportOffset + 15] << 8
                            | rxPacketBuffer[currReportOffset + 14]);

                    gameRotationVector = new Quaternion(
                            qToFloat(realPartQ, ROTATION_Q_POINT),
                            qToFloat(data1, ROTATION_Q_POINT),
                            qToFloat(data2, ROTATION_Q_POINT),
                            qToFloat(data3, ROTATION_Q_POINT));

                    currReportOffset += SIZEOF_GAME_ROTATION_VECTOR;
                }
                    break;

                default:
                    // some report we don't understand, so bail
                    return;
            }

            if (currReportOffset >= SHTP_RX_PACKET_SIZE) {
                return;
            }
        }

    }

    /**
     * Read a packet if it's waiting.
     *
     * @param channel  Channel of the packet
     * @param reportID Report ID (first data byte) of the packet
     * @return true if the packet has been received
     */
    public boolean readPacket(int channel, int reportID) {

        if (!receivePacket()) {
            return false;
        }

        if (channel == rxPacketBuffer[2] && reportID == rxPacketBuffer[4]) {
            // found correct packet!
            return true;
        } else {
            // other data packet, send to proper channels
            processPacket();
        }

        return false;
    }

    /**
     * Given a Q value, converts fixed point floating to regular floating point
     * number.
     * See https://en.wikipedia.org/wiki/Q_(number_format)
     * 
     * @param fixedPointValue
     * @param qPoint
     * @return
     */
    private float qToFloat(short fixedPointValue, int qPoint) {
        float qFloat = fixedPointValue;
        qFloat *= Math.pow(2.0f, qPoint * -1);
        return (qFloat);
    }

    /**
     * Given a sensor's report ID, this tells the BNO080 to begin reporting the
     * values.
     *
     * @param reportID
     * @param timeBetweenReports
     * @param specificConfig     the specific config word. Useful for personal
     *                           activity classifier.
     */
    private void setFeatureCommand(byte reportID, int timeBetweenReports) {
        int specificConfig = 0;
        int microsBetweenReports = (int) (timeBetweenReports * 1000);

        int batchMicros = 0;

        txPacketBuffer[4] = (byte) SHTP_REPORT_SET_FEATURE_COMMAND; // Set feature command. Reference page 55
        txPacketBuffer[5] = (byte) reportID; // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
        txPacketBuffer[6] = (byte) 0; // Feature flags
        txPacketBuffer[7] = (byte) 0; // Change sensitivity (LSB)
        txPacketBuffer[8] = (byte) 0; // Change sensitivity (MSB)
        txPacketBuffer[9] = (byte) ((microsBetweenReports >> 0) & 0xFF); // Report interval (LSB) in microseconds.
                                                                         // 0x7A120 = 500ms
        txPacketBuffer[10] = (byte) ((microsBetweenReports >> 8) & 0xFF); // Report interval
        txPacketBuffer[11] = (byte) ((microsBetweenReports >> 16) & 0xFF); // Report interval
        txPacketBuffer[12] = (byte) ((microsBetweenReports >> 24) & 0xFF); // Report interval (MSB)
        txPacketBuffer[13] = (byte) ((batchMicros >> 0) & 0xFF); // Batch Interval (LSB)
        txPacketBuffer[14] = (byte) ((batchMicros >> 8) & 0xFF); // Batch Interval
        txPacketBuffer[15] = (byte) ((batchMicros >> 16) & 0xFF);// Batch Interval
        txPacketBuffer[16] = (byte) ((batchMicros >> 24) & 0xFF);// Batch Interval (MSB)
        txPacketBuffer[17] = (byte) ((specificConfig >> 0) & 0xFF); // Sensor-specific config (LSB)
        txPacketBuffer[18] = (byte) ((specificConfig >> 8) & 0xFF); // Sensor-specific config
        txPacketBuffer[19] = (byte) ((specificConfig >> 16) & 0xFF); // Sensor-specific config
        txPacketBuffer[20] = (byte) ((specificConfig >> 24) & 0xFF); // Sensor-specific config (MSB)
        sendPacket(CHANNEL_CONTROL, 17);
    }

    /**
     * Reads a packet from the IMU (if any are waiting) and stores it in the class
     * variables.
     *
     * The USC version of this has a wait and a timeout, which is never what we
     * want.
     * 
     * @return whether a packet was recieved.
     */
    public boolean receivePacket() {
        // first read the header.
        m_i2c.readOnly(rxPacketBuffer, 4);

        // // Get the first four bytes, aka the packet header
        // byte packetLSB = (byte) (m_i2c.read(true));
        // byte packetMSB = (byte) (m_i2c.read(true));
        // byte channelNumber = (byte) (m_i2c.read(true));
        // byte sequenceNum = (byte) (m_i2c.read(true)); // Not sure if we need to store
        // this or not

        // // Store the header info
        // rxPacketBuffer[0] = packetLSB;
        // rxPacketBuffer[1] = packetMSB;
        // rxPacketBuffer[2] = channelNumber;
        // rxPacketBuffer[3] = sequenceNum;

        byte packetLSB = rxPacketBuffer[0];
        byte packetMSB = rxPacketBuffer[1];

        if (packetLSB == 0xFF && packetMSB == 0xFF) {
            // invalid according to BNO080 datasheet section 1.4.1
            return false;
        }

        // Calculate the number of data bytes in this packet
        rxPacketLength = (packetMSB << 8) | packetLSB;

        // Clear the MSbit.
        // This bit indicates if this package is a continuation of the last. TBH, I
        // don't really know what this means (it's not really explained in the
        // datasheet)
        // but we don't actually care about any of the advertisement packets
        // that use this, so we can just cut off the rest of the packet by releasing
        // chip select.
        rxPacketLength &= ~(1 << 15);

        if (rxPacketLength == 0) {
            // Packet is empty
            return false;
        }
        // Remove the header bytes from the data count since we already read them
        rxPacketLength -= 4;
        // avoid overflow
        rxPacketLength = Math.min(rxPacketLength, rxPacketBuffer.length - 4);

        for (int i = 0; i < rxPacketLength - 4; ++i) {
            byte[] inbound = new byte[1];
            m_i2c.readOnly(inbound, 1);
            rxPacketBuffer[i + 4] = inbound[0];
        }

        return true;
    }

    /**
     * Sends the current shtpData contents to the BNO. It's a good idea to disable
     * interrupts before you call this.
     *
     * Given the data packet, send the header then the data
     * Returns false if sensor does not ACK
     * 
     * @param channelNumber the channel to send on
     * @param dataLength    How many bits of shtpData to send
     * @return true if write succeeds
     */

    private boolean sendPacket(int channelNumber, int dataLength) {

        int totalLength = dataLength + 4; // Add four bytes for the header

        txPacketBuffer[0] = (byte) (totalLength & 0xFF);
        txPacketBuffer[1] = (byte) (totalLength >> 8);
        txPacketBuffer[2] = (byte) channelNumber;
        txPacketBuffer[3] = (byte) 0;

        // send packet to IMU
        boolean writeResult = m_i2c.writeBulk(txPacketBuffer, totalLength);

        if (writeResult) {
            System.out.println("BNO I2C write failed!\n");
            return false;
        }
        return true;
    }

    /**
     * Erases the current SHTP TX packet buffer.
     * In BNO080Async, this blocks until the buffer is available.
     */
    private void clearSendBuffer() {
        Arrays.fill(txPacketBuffer, (byte) 0);
    }

}
