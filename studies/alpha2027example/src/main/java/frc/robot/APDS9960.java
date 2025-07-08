package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * See
 * https://cdn.sparkfun.com/assets/learn_tutorials/3/2/1/Avago-APDS-9960-datasheet.pdf
 */
public class APDS9960 {
    private static final byte ADDR = (byte) 0x39;
    private static final byte ENABLE = (byte) 0x80;
    private static final byte APDS9960_PDATA = (byte) 0x9C;

    private final I2C m_i2c;

    public APDS9960() {
        m_i2c = new I2C(I2C.Port.kPort1, ADDR);
        // turn the device on and enable proximity measurement
        m_i2c.write(ENABLE, 0b00000101);
    }

    public int read() {
        byte[] buf = new byte[1];
        m_i2c.read(APDS9960_PDATA, 1, buf);
        return Byte.toUnsignedInt(buf[0]);
    }

}
