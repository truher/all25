package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class AS5048B_I2C {
    private static final byte ADDR = (byte) 0x40;
    private static final byte ANGLMSB_REG = (byte) 0xFE;
    private final I2C m_i2c;

    public AS5048B_I2C() {
        m_i2c = new I2C(I2C.Port.kPort0, ADDR);
    }

    public int get() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(ANGLMSB_REG, 2, buf);
        int readValue = buf.get() << 6;
        readValue += buf.get() & 0x3F;
        return readValue;
    }

}
