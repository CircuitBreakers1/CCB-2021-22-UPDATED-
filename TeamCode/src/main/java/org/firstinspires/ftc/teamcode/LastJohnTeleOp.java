/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Robot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Robot2023.imu;
import static org.firstinspires.ftc.teamcode.Robot2023.intake;
import static org.firstinspires.ftc.teamcode.Robot2023.leftBack;
import static org.firstinspires.ftc.teamcode.Robot2023.leftFront;
import static org.firstinspires.ftc.teamcode.Robot2023.lift;
import static org.firstinspires.ftc.teamcode.Robot2023.liftRaise;
import static org.firstinspires.ftc.teamcode.Robot2023.rightBack;
import static org.firstinspires.ftc.teamcode.Robot2023.rightFront;
import static org.firstinspires.ftc.teamcode.Robot2023.shotRelease;
import static org.firstinspires.ftc.teamcode.Robot2023.wrist;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Demonstrates new features
 */
@TeleOp(name = "TeleOP", group = "")
public class LastJohnTeleOp extends OpMode {

    Robot2023 robot = new Robot2023();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, false);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        robot.smoothDrive(x, y, gamepad1.right_stick_x);

        if(gamepad1.a || gamepad2.a) {
            intake.setPower(-1);
        } else if (gamepad1.b || gamepad2.b) {
            intake.setPower(0);
        }

        if(gamepad1.x) {
            shotRelease.setPosition(0);
        } else if (gamepad1.y) {
            shotRelease.setPosition(0.1);
        }

        if(gamepad2.x) {
            gripper.setPosition(0);
        } else if (gamepad2.y) {
            gripper.setPosition(1);
        }

        if (gamepad2.dpad_up) {
            lift.setPower(1);
        } else if (gamepad2.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad2.right_bumper) {
            wrist.setPosition(0);
        } else if (gamepad2.left_bumper) {
            wrist.setPosition(1);
        }

        if(gamepad2.dpad_left) {
            liftRaise.setPosition(0);
        } else if (gamepad2.dpad_right) {
            liftRaise.setPosition(0.1);
        }

        armAngle.setPower(-gamepad2.left_stick_y);
        armExtend.setPower(gamepad2.right_stick_y);

        telemetry.addData("Arm Length:", armExtend.getCurrentPosition());
        telemetry.update();
    }
}
