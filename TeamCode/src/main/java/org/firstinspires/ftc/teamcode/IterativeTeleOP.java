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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.ButtonToggle.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TeleOP")
public class IterativeTeleOP extends OpMode {
    Robot robot = new Robot(this, false);
    int resetPhase = 0;
    boolean isIntaking = false;
    boolean isOutputting = false;
    boolean isResetting = false;

    //Telemetry.Item intake = telemetry.addData("Is intaking?", isIntaking);
    //Telemetry.Item output = telemetry.addData("Is outputting?", isOutputting);
    //Telemetry.Item touchSense = telemetry.addData("Touch Sensor", touch.getState());
    //Telemetry.Item armPos = telemetry.addData("Arm Position", angle.getCurrentPosition());


    @Override
    public void init() {
        robot.init(hardwareMap);
        ButtonToggle.opMode = this;
    }

    @Override
    public void init_loop() {
        updateButtons();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (!isResetting) {

            updateButtons();

            //telemetry.setAutoClear(false); //Stuff will need to be manually removed
            //telemetry.addAction(new Runnable() { @Override public void run() {updateLocation();} });
            //telemetry.addData("Robot X", ".3f%", xLoc)
            //      .addData(" Robot Y", ".3f%", yLoc)
            //    .addData(" Robot Angle", ".3f%", rotation);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //angle.setPower(-gamepad2.left_stick_y);

            float rangeGetter = angle.getCurrentPosition();

            if (-gamepad2.left_stick_y > 0 && (rangeGetter <= 4300 || gamepad2.start)) {
                angle.setPower(-gamepad2.left_stick_y);
            } else if (-gamepad2.left_stick_y < 0 && (rangeGetter >= -800 || gamepad2.start)) {
                angle.setPower(0.5 * -gamepad2.left_stick_y);
            } else {
                angle.setPower(0);
            }
            //Max 4300
            //Min -990


            float extensionGetter = extension.getCurrentPosition();
            if (gamepad2.right_stick_y < 0 && (extensionGetter >= -3000 || gamepad2.start)) {
                extension.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0 && (extensionGetter < 0 || gamepad2.start)) {
                extension.setPower(gamepad2.right_stick_y);
            } else {
                extension.setPower(0);
            }


            Robot.leftFront.setPower(0.75 * (y + x + rx));
            Robot.leftBack.setPower(0.75 * (y - x + rx));
            Robot.rightFront.setPower(0.75 * (y - x - rx));
            Robot.rightBack.setPower(0.75 * (y + x - rx));

            if (gamepad2.a) {
                leftSuck.setPower(1);
                rightSuck.setPower(-1);
                isIntaking = true;
                isOutputting = false;
            } else if (gamepad2.b) {
                leftSuck.setPower(0);
                rightSuck.setPower(0);
                isIntaking = false;
                isOutputting = false;
            } else if (gamepad2.x) {
                leftSuck.setPower(-1);
                rightSuck.setPower(1);
                isIntaking = false;
                isOutputting = true;
            }

            //touchSense.setValue(touch.getState());
            //intake.setValue(isIntaking);
            //output.setValue(isOutputting);
            //armPos.setValue(angle.getCurrentPosition());

            telemetry.addData("Arm Position", angle.getCurrentPosition());


            if (!touch.getState() && isIntaking) {
                leftSuck.setPower(0);
                rightSuck.setPower(0);
                isIntaking = false;
                isOutputting = false;
            }
        }

        if (gamepad2.back && gamepad2.start && !isResetting) {
            resetPhase = 0;
            isResetting = true;
            angle.setTargetPosition(600);
            angle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            angle.setPower(0.5);
        }

        if (isResetting) {
            switch (resetPhase) {
                case 0:
                    if (!angle.isBusy()) {
                        angle.setPower(0);
                        extension.setTargetPosition(0);
                        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extension.setPower(1);
                        resetPhase = 1;
                    }
                    break;
                case 1:
                    if (!extension.isBusy()) {
                        extension.setPower(0);
                        angle.setTargetPosition(0);
                        angle.setPower(0.5);
                        resetPhase = 2;
                    }
                    break;
                case 2:
                    if (!angle.isBusy()) {
                        angle.setPower(0);

                        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        isResetting = false;
                    }
                    break;
            }
        }
    }

    @Override
    public void stop() {
        //Make sure the robot controller clears any actions setup by the telemetry so they do not
        //run when the opmode has stopped
        telemetry.clearAll();
    }

}
