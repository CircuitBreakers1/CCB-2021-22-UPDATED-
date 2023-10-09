package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(name = "NeoDriver", xmlTag = "AdafruitNeoDriver")
public class NeoDriverI2C extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private final boolean isRGBW;
    private final int numPixels;

    protected NeoDriverI2C(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned, boolean isRGBW, int numPixels) {
        super(i2cDeviceSynch, deviceClientIsOwned);

        this.isRGBW = isRGBW;
        this.numPixels = numPixels;

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x30));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit NeoDriver";
    }

    private enum Register {
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05);

        public final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void writeShort(Register reg, short val, ByteOrder byteOrder) {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(val, byteOrder));
    }

    protected void writeShort(Register reg, short val) {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(val));
    }

    public void init() {
        //Set the "pin" used by the seesaw firmware
        writeShort(Register.PIN, (short) 0xF);
        //Set the I2C Speed
        writeShort(Register.SPEED, (short) 0x1, ByteOrder.LITTLE_ENDIAN);
        //Set the buffer length
        if(isRGBW) {
            writeShort(Register.BUF_LENGTH, (short) 0x4);
        } else {
            writeShort(Register.BUF_LENGTH, (short) 0x3);
        }
    }

    /**
     * Sets the color of multiple pixels
     * @param startPixel The pixel to begin writing (Inclusive)
     * @param endPixel The pixel to stop writing (Inclusive)
     * @param color The color to use
     */
    public void setPixels(int startPixel, int endPixel, Color color) {
        if (startPixel < 0 || endPixel >= numPixels) {
            throw new IllegalArgumentException("Pixel range out of bounds");
        }

        int pixels = endPixel - startPixel + 1;

        for (int i = 0; i < pixels; i += 9) {
            short startAddr = (short) (i + startPixel);
            int maxWriteThisBatch = Math.min(pixels - i, 9);
            byte[] data = new byte[(maxWriteThisBatch * 3) + 2];
            byte[] strtAddr = TypeConversion.shortToByteArray(startAddr);
            data[0] = strtAddr[0];
            data[1] = strtAddr[1];

            for (int j = 0; j < maxWriteThisBatch; j++) {
                data[2 + (j * 3)] = (byte) color.r;
                data[3 + (j * 3)] = (byte) color.g;
                data[4 + (j * 3)] = (byte) color.b;
            }

            this.deviceClient.write(Register.BUF.bVal, data);
            this.deviceClient.write(Register.SHOW.bVal, new byte[] {0x1});
        }
    }

    public void setAllPixels(Color color) {
        setPixels(0, numPixels - 1, color);
    }

    public class Color {
        public int r;
        public int g;
        public int b;

        public Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}
