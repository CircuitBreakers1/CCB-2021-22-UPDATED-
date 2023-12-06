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
        neoDriver.setNumPixels(5);
        neoDriver.setRGBW(false);
        neoDriver.init();
        //neoDriver.setAllPixels(new NeoDriverI2C.Color(255, 0, 0));
        int debug = neoDriver.test();
        telemetry.addData("Write Length", debug);
        telemetry.addData("Manufacturer ID", neoDriver.getManufacturer().name());
    }

    @Override
    public void loop() {

    }
}
