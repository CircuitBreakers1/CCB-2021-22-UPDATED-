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
    private short numPixels;

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

    public void setNumPixels(short numPixels) {
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
        clear();
    }

    /**
     * Sets the color of multiple pixels
     *
     * @param endPixel The pixel to stop writing (Inclusive)
     * @param color The color to use
     */
    public void setPixels(short endPixel, Color color) {
        if (endPixel >= numPixels) {
            throw new IllegalArgumentException("Pixel range out of bounds");
        }
        byte[] addr = TypeConversion.shortToByteArray((short) 0);
        clear(endPixel);

        //Clear all pixels in range by setting them to black

        byte[] output = new byte[2 + 3 * (endPixel + 1)];
        output[0] = addr[0];
        output[1] = addr[1];
        for(int i = 2; i < output.length; i += 3) {
            output[i] = color.g;
            output[i + 1] = color.r;
            output[i + 2] = color.b;
        }

        writeBytes(Register.BUF, output);
        writeBytes(Register.SHOW, new byte[] {0x1});
        writeBytes(Register.SHOW, new byte[] {0x0});
    }

    public void setAllPixels(Color color) {
        setPixels((short) (numPixels - 1), color);
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
        short startAddr = 3;
        byte[] startAddrBytes = TypeConversion.shortToByteArray(startAddr);
        byte[] data = new byte[] {startAddrBytes[0], startAddrBytes[1], 0x00, (byte) 0xFF, 0x00};
        writeBytes(Register.BUF, data);
        writeBytes(Register.SHOW, new byte[] {0x1});
        writeBytes(Register.SHOW, new byte[] {0x0});
        return data.length;
    }

    public void clear() {
        clear((short) (numPixels - 1));
    }

    public void clear(int end) {
        byte[] addr = TypeConversion.shortToByteArray((short) 0);
        byte[] clear = new byte[2 + 3 * (end + 1)];
        clear[0] = addr[0];
        clear[1] = addr[1];
        for(int i = 2; i < clear.length; i += 3) {
            clear[i] = 0x0;
            clear[i + 1] = 0x0;
            clear[i + 2] = 0x0;
        }

        writeBytes(Register.BUF, clear);
        writeBytes(Register.SHOW, new byte[] {0x1});
        writeBytes(Register.SHOW, new byte[] {0x0});
    }
}
