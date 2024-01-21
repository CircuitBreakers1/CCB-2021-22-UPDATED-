package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.NeoDriverI2C;

@TeleOp(name = "Neo Test", group = "")
public class NeoDriverTest extends OpMode {
    NeoDriverI2C neoDriver;
    @Override
    public void init() {
        neoDriver = hardwareMap.get(NeoDriverI2C.class, "neoDriver");
        neoDriver.setNumPixels((short) 10);
        neoDriver.setRGBW(false);
        neoDriver.init();
//        neoDriver.test();
        //neoDriver.setAllPixels(new NeoDriverI2C.Color((short) 255, (short) 0, (short) 0));
        neoDriver.setPixels((short) 9, new NeoDriverI2C.Color((short) 0, (short) 255, (short) 0));
        neoDriver.setPixels((short) 4, new NeoDriverI2C.Color((short) 255, (short) 0, (short) 0));
////        int debug = neoDriver.test();
//        byte[] test2 = {test[0], test[1]};
//        telemetry.addData("Write Spot", test2[0] + " " + test2[1]);
//        telemetry.addData("Manufacturer ID", neoDriver.getManufacturer().name());
    }

    @Override
    public void loop() {

    }

    public void stop() {
        neoDriver.clear();
    }
}
