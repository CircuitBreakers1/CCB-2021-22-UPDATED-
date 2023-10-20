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

package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.teamcode.archive.SummerJohn2023.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archive.SummerJohn2023;

@TeleOp(name = "Old TeleOP")
@Disabled
public class SummerTeleOP extends LinearOpMode {
    SummerJohn2023 robot = new SummerJohn2023();

    //Constants
    final int armLevels[] = {500, 1000, 1500};
    final int armMax = 4000;
    final int armRotateLevel = 500;
    boolean yes = false, yes2 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        leftGrab.setPosition(0.15); //Closed 0.1, Open 0.15
        rightGrab.setPosition(0.1); //Closed 0.15, Open 0.1
//        sleep(1000);
//        rightGrab.setPosition(0.3);
//        sleep(1000);
//        rightGrab.setPosition(0.5);
//        sleep(1000);
//        rightGrab.setPosition(0.7);
//        sleep(1000);
//        rightGrab.setPosition(0.9);



        sleep(1000);


        rotate.setPower(0.5);
        while(rotateTrigger.getState()) {
            telemetry.addData("Status", "Zeroing...");
            telemetry.update();
        }
        rotate.setPower(0);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setTargetPosition(0);

        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftLift.setTargetPosition(0);
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.setArmHeight(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        leftLift.setPower(0.2);
//        //rightLift.setPower(0.2);
//
//        sleep(1000);
//
//        leftLift.setPower(0);
//        rightLift.setPower(0);
//
//        sleep(500);
//
//        leftLift.setPower(-0.2);
//        //rightLift.setPower(-0.2);
//
//        sleep(1000);

        leftLift.setPower(0);
        rightLift.setPower(0);

        //Set the first gamepad led to purple, and the second to yellow
        gamepad1.setLedColor(255, 0, 255, -1);
        gamepad2.setLedColor(255, 255, 0, -1);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double heading = 69;

            double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
            double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

            leftFront.setPower(1 * (y + x + rx));
            leftBack.setPower(1 * (y - x + rx));
            rightFront.setPower(1 * (y - x - rx));
            rightBack.setPower(1 * (y + x - rx));




            if(leftLift.getCurrentPosition() >= armRotateLevel || Math.abs(rotate.getCurrentPosition()) - 40 < 0) {
                if((gamepad2.left_stick_x < 0 && rotate.getCurrentPosition() <= 470) || (gamepad2.left_stick_x > 0 && rotate.getCurrentPosition() >= -470)) {
                    rotate.setPower(-gamepad2.left_stick_x * 0.25);
                } else {
                    rotate.setPower(0);
                }
            } else {
                rotate.setPower(0);
            }
            /* else if (Math.abs(rotate.getCurrentPosition()) > 100) {
                //noinspection IntegerDivisionInFloatingPointContext
                rotate.setPower(Math.min(0.1 * (rotate.getCurrentPosition() / Math.abs(rotate.getCurrentPosition())), 0.25));
            } */

//            if(gamepad2.a) {
//                robot.setArmHeight(armLevels[0]);
//            } else if(gamepad2.b) {
//                robot.setArmHeight(armLevels[1]);
//            } else if(gamepad2.y) {
//                robot.setArmHeight(armLevels[2]);
//            } else if(gamepad2.x) {
//                robot.setArmHeight(0);
//            }


            yes = (-gamepad2.right_stick_y > 0 && leftLift.getCurrentPosition() < armMax);
            yes2 = -gamepad2.right_stick_y < 0 && (Math.abs(rotate.getCurrentPosition()) < 75 || leftLift.getCurrentPosition() > armRotateLevel);

            if(yes || yes2) {
                leftLift.setPower(-gamepad2.right_stick_y);
                rightLift.setPower(gamepad2.right_stick_y);

            } else if (gamepad2.right_stick_y == 0) {
                if(leftLift.getCurrentPosition() > 1000) {
                    leftLift.setPower(0.1);
                    rightLift.setPower(-0.1);
                } else {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                }
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }


            //Prevent changing arm power if leftlift position is above armMax


//            leftLift.setPower(-gamepad2.right_stick_y);
//            rightLift.setPower(gamepad2.right_stick_y);

//
//            if(gamepad2.dpad_up) {
//                leftLift.setPower(0.5);
//                rightLift.setPower(-0.5);
//            }else if(gamepad2.dpad_down) {
//                leftLift.setPower(-0.5);
//                rightLift.setPower(.5);
//            } else {
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//            }

            if(gamepad2.left_bumper) {
                //Close
                rightGrab.setPosition(0.17);
                leftGrab.setPosition(0.08);
            }
            if(gamepad2.right_bumper) {
                //Open
                rightGrab.setPosition(0.1);
                leftGrab.setPosition(0.15);
            }

            //robot.maintainArm();

            telemetry.addData("Left Arm Speed", leftLift.getVelocity());
            telemetry.addData("Right Arm Speed", rightLift.getVelocity());
            telemetry.addData("Arm Height", leftLift.getCurrentPosition());
            telemetry.addData("Turret Location", rotate.getCurrentPosition());
            telemetry.addData("Inverted Gamepad", -gamepad2.right_stick_y);
            telemetry.addData("Left Pos", leftGrab.getPosition());
            telemetry.addData("YES?", yes);
            telemetry.addData("YES2?", yes2);
            telemetry.update();
        }
    }
}
