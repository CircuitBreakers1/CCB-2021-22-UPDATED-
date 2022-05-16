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

package org.firstinspires.ftc.teamcode.archive.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * This file is the main code for our TeleOP
 */

@TeleOp(name = "oldTeleOP", group = "Linear Opmode")
@Disabled
public class TeleOP extends LinearOpMode {

    HardwareInit robot = new HardwareInit();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //Various booleans used for toggle buttons
        boolean spinOn = false;
        boolean rightBumperDown = false;
        boolean leftBumperDown = false;

        boolean intakeOn = false;
        boolean outputOn = false;

        boolean manualControl = false;
        boolean armDown = true;

        waitForStart();

        while (opModeIsActive()) {
            // Simple tank drive
            robot.leftFront.setPower(-gamepad1.left_stick_y);
            robot.leftBack.setPower(-gamepad1.left_stick_y);
            robot.rightBack.setPower(-gamepad1.right_stick_y);
            robot.rightFront.setPower(-gamepad1.right_stick_y);

            if (gamepad2.right_bumper) {
                if (!rightBumperDown) {
                    rightBumperDown = true;
                    if (spinOn) {
                        robot.backSpinner.setPower(0);
                        spinOn = false;
                    } else {
                        robot.backSpinner.setPower(-0.7);
                        spinOn = true;
                    }
                }
            } else {
                rightBumperDown = false;
            }

            if (gamepad2.left_bumper) {
                if (!leftBumperDown) {
                    leftBumperDown = true;
                    if (spinOn) {
                        robot.backSpinner.setPower(0);
                        spinOn = false;
                    } else {
                        robot.backSpinner.setPower(0.7);
                        spinOn = true;
                    }
                }
            } else {
                leftBumperDown = false;
            }

            /*
             * The next couple if blocks allow the arm to be easily set to each level
             */
            if (gamepad2.a) {
                // Bottom Level
                robot.rightArm.setTargetPosition(110);
                robot.leftArm.setTargetPosition(110);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
                armDown = false;
            }

            if (gamepad2.b) {
                // Middle Level
                robot.rightArm.setTargetPosition(215);
                robot.leftArm.setTargetPosition(215);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
                armDown = false;
            }

            if (gamepad2.y) {
                // Top Level
                robot.rightArm.setTargetPosition(318);
                robot.leftArm.setTargetPosition(318);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
                armDown = false;
            }

            if (gamepad2.x) {
                // Bring to rest
                robot.rightArm.setTargetPosition(0);
                robot.leftArm.setTargetPosition(0);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(0.25);
                robot.leftArm.setPower(0.25);
                armDown = true;
            }

            if(armDown && robot.leftArm.getCurrentPosition() < 6) {
                robot.leftArm.setPower(0);
                robot.rightArm.setPower(0);

                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
            }

            /*
             * This block of code allows the stick to be used to adjust the position of the arm
             * even while it is running to a position. Once the desired position is reached, releasing
             * the stick will have it running to the position you left it to try and stay there.
             */
            if (gamepad2.left_stick_y != 0) {
                if (!manualControl) {
                    manualControl = true;
                    robot.rightArm.setPower(0);
                    robot.leftArm.setPower(0);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                robot.leftArm.setPower(-gamepad2.left_stick_y * .5);
                robot.rightArm.setPower(-gamepad2.left_stick_y * .5);
            } else if (manualControl) {
                manualControl = false;
                robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition());
                robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition());

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
            }


            // A touch sensor located where the arm bottoms out allows the encoders to zero out and
            // ensure no encoder errors build up over time
            if (!robot.touchSensor.getState()) {
                robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad2.dpad_up) {
                robot.intake.setPower(-1);
                robot.rightArm.setTargetPosition(0);
                robot.leftArm.setTargetPosition(0);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
                intakeOn = true;
                outputOn = false;
            }
            if(intakeOn && !robot.cargoTouch.getState()) {
                robot.intake.setPower(0);

                robot.rightArm.setPower(0);
                robot.leftArm.setPower(0);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                intakeOn = false;
                outputOn = false;
            }
            if(gamepad2.dpad_down) {
                robot.intake.setPower(1);
                intakeOn = false;
                outputOn = true;
            }
            if(gamepad2.dpad_left) {
                robot.intake.setPower(0);

                robot.rightArm.setPower(0);
                robot.leftArm.setPower(0);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                intakeOn = false;
                outputOn = false;
            }

            if(intakeOn) {
                setLEDs(!robot.cargoTouch.getState(), true);
            } else setLEDs(outputOn, false);

            // Telemetry code for showing encoder values. Helpful during debugging.
            telemetry.addData("Left Arm Position", robot.leftArm.getCurrentPosition());
            telemetry.addData("Right Arm Position", robot.rightArm.getCurrentPosition());
            telemetry.update();

        }
    }

    /**
     * This function serves as an easy way to set all the LED indicators.
     */
    public void setLEDs(boolean red, boolean green) {
        robot.leftLEDGreen.setState(green);
        robot.leftLEDRed.setState(red);
        robot.rightLEDGreen.setState(green);
        robot.rightLEDRed.setState(red);
    }
}