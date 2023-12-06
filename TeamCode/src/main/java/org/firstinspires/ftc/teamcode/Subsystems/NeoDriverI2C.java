package org.firstinspires.ftc.teamcode.Subsystems;


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

    private boolean isRGBW;
    private int numPixels;

    public NeoDriverI2C(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x60));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    public void setNumPixels(int numPixels) {
        this.numPixels = numPixels;
    }

    public void setRGBW(boolean isRGBW) {
        this.isRGBW = isRGBW;
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
        FIRST(0),
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05),
        LAST(SHOW.bVal);

        public byte bVal;

        Register(int bVal) {
            this.bVal = (byte) bVal;
        }
    }

    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeBytes(Register reg, byte[] val) {

        byte[] output = new byte[val.length + 2];

        output[0] = 0x0E;
        output[1] = reg.bVal;

        System.arraycopy(val, 0, output, 2, val.length);

        this.deviceClient.write(output);
    }

    protected void writeByteLE(Register reg, byte val) {
        writeBytes(reg, TypeConversion.shortToByteArray(val, ByteOrder.LITTLE_ENDIAN));
    }

    protected void writeShort(Register reg, short val, ByteOrder byteOrder) {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(val, byteOrder));
    }

    protected void writeShort(Register reg, short val) {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(val));
    }

    public void init() {
        //Set the "pin" used by the seesaw firmware
        writeBytes(Register.PIN, new byte[] {0xF});
        //writeShort(Register.PIN, (short) 0xF);
        //Set the I2C Speed
        writeBytes(Register.SPEED, new byte[] {0x00});
        //writeShort(Register.SPEED, (short) 0x1, ByteOrder.LITTLE_ENDIAN);
        //Set the buffer length
        if(isRGBW) {
            writeByteLE(Register.BUF_LENGTH, (byte) 0x04);
        } else {
            writeByteLE(Register.BUF_LENGTH, (byte) 0x03);
        }

        writeBytes(Register.SHOW, new byte[] {0x0});
    }

    /**
     * Sets the color of multiple pixels
     *
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

            writeBytes(Register.BUF, data);
            writeBytes(Register.SHOW, new byte[] {0x1});
        }
    }

    public void setAllPixels(Color color) {
        setPixels(0, numPixels - 1, color);
    }

    public static class Color {
        public byte r;
        public byte g;
        public byte b;

        //Write order is GRB
        public Color(short r, short g, short b) {
            this.r = (byte) r;
            this.g = (byte) g;
            this.b = (byte) b;
        }
    }

    //Set the color of the first pixel to red.
    public int test() {
        short startAddr = 0;
        byte[] startAddrBytes = TypeConversion.shortToByteArray(startAddr);
        byte[] data = new byte[] {startAddrBytes[0], startAddrBytes[1], 0x00, (byte) 0xFF, 0x00};
        writeBytes(Register.BUF, data);
        writeBytes(Register.SHOW, new byte[] {0x1});
        writeBytes(Register.SHOW, new byte[] {0x0});
        return data.length;
    }
}
