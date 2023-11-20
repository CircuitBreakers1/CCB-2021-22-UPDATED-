package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@TeleOp(name = "Camera Test", group = "")
public class CameraTest extends OpMode {

    Robot2023 robot = new Robot2023();

    @Override
    public void init() {
        robot.init(hardwareMap, true, null);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Color Guess", robot.cameraSubsystem.getPropGuess());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Color Guess", robot.cameraSubsystem.getPropGuess());
    }
}
