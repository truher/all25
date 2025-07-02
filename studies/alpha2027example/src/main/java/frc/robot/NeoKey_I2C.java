package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * see https://github.com/adafruit/Adafruit_Seesaw/blob/master/Adafruit_NeoKey_1x4.cpp
 */
public class NeoKey_I2C {
    private final I2C m_i2c;

    public NeoKey_I2C() {
        m_i2c = new I2C(I2C.Port.kPort1, (byte) 0x30);
    }

    public byte[] read() {
        int reg = 0x0004;
        byte[] buf = new byte[4];
        m_i2c.read(reg, 4, buf);
        return buf;
    }
}
