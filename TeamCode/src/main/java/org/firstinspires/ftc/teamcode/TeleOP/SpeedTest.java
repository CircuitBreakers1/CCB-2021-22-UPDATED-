package org.firstinspires.ftc.teamcode.TeleOP;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@TeleOp(name = "Speed Test", group = "")
public class SpeedTest extends OpMode {
    @Override
    public void init() {
        robot.init(hardwareMap, false, null);
    }

    Robot2023 robot = new Robot2023();
    long timestamp = 0;
    int avgCount = 0;
    long total;
    long current;

    @Override
    public void start() {
        timestamp = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        //10 ms
        holOdom.updatePose();
        current = System.currentTimeMillis() - timestamp;
        timestamp = System.currentTimeMillis();
        avgCount++;
        total += current;
        telemetry.addData("Last Time", current);
        telemetry.addData("Average", total / avgCount);
    }
}
