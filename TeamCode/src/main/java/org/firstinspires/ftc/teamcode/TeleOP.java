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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file is the main code for our TeleOP
 */


/*
 * Important Values:
 * Grabbing Servos:
 * Closed: Left 1.0, Right 0.0
 * Open: Left 0.7, Right 0.3
 * Encoder Arm Height:
 * Bottom Level: 51
 * Middle Level: 90
 * Top Level: 140
 * <p>
 * Notes:
 * Grabbing Servos:
 * Open: LEDs Green
 * Closed: LEDs Red
 */

@TeleOp(name = "TeleOP", group = "Linear Opmode")
public class TeleOP extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor backSpinner;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private Servo leftGrabber;
    private Servo rightGrabber;
    private DigitalChannel touchSensor;
    private DigitalChannel leftLEDGreen;
    private DigitalChannel leftLEDRed;
    private DigitalChannel rightLEDGreen;
    private DigitalChannel rightLEDRed;
    private DistanceSensor distance;


    @Override
    public void runOpMode() {
        // Set the hardware paths for all of our actuators, sensors, and other things connected to I/O
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        backSpinner = hardwareMap.dcMotor.get("backSpinner");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");

        rightGrabber = hardwareMap.servo.get("rightGrabber");
        leftGrabber = hardwareMap.servo.get("leftGrabber");

        touchSensor = hardwareMap.digitalChannel.get("touchSensor");
        leftLEDGreen = hardwareMap.digitalChannel.get("leftLEDGreen");
        leftLEDRed = hardwareMap.digitalChannel.get("leftLEDRed");
        rightLEDGreen = hardwareMap.digitalChannel.get("rightLEDGreen");
        rightLEDRed = hardwareMap.digitalChannel.get("rightLEDRed");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // The digital channel defaults to inputs, so we have to set the LEDs channels to outputs
        leftLEDRed.setMode(DigitalChannel.Mode.OUTPUT);
        leftLEDGreen.setMode(DigitalChannel.Mode.OUTPUT);
        rightLEDRed.setMode(DigitalChannel.Mode.OUTPUT);
        rightLEDGreen.setMode(DigitalChannel.Mode.OUTPUT);

        /*
         * Some motors, due to positioning require to be reversed, in the case of the wheels, to
         * make the wheels move forward when given positive power
         */
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        //Sets the zero power behavior of most motors to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        //Various booleans used for toggle buttons
        boolean spinOn = false;
        boolean aDown = false;

        boolean grabOpen = true;
        boolean triggerDown = false;

        boolean manualControl = false;


        // Left 1 is down, right 0 is down
        // Start Open
        rightGrabber.setPosition(0.3);
        leftGrabber.setPosition(0.7);
        // Set the LEDs green to show the grabbing servos are open
        setLEDs(true, true);


        waitForStart();

        while (opModeIsActive()) {
            // Simple tank drive
            leftFront.setPower(-gamepad1.left_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
            rightBack.setPower(-gamepad1.right_stick_y);
            rightFront.setPower(-gamepad1.right_stick_y);

            /*
            leftArm.setPower(-gamepad2.left_stick_y * .5);
            rightArm.setPower(-gamepad2.left_stick_y * .5);
            */

            if(distance.getDistance(DistanceUnit.INCH) < 1.5 && grabOpen) {
                setLEDs(false,true);
            } else if (grabOpen) {
                setLEDs(true, true);
            } else {
                setLEDs(true, false);
            }

            /*
             * This block of code allows the A button on the driver controller to toggle the
             * carousel spinner on and off.
             */
            if (gamepad1.a) {
                if (!aDown) {
                    aDown = true;
                    if (spinOn) {
                        backSpinner.setPower(0);
                        spinOn = false;
                    } else {
                        backSpinner.setPower(-0.65);
                        spinOn = true;
                    }
                }
            } else {
                aDown = false;
            }

            /*
             * The next couple if blocks allow the arm to be easily set to
             */
            if (gamepad2.a) {
                // Bottom Level
                rightArm.setTargetPosition(51);
                leftArm.setTargetPosition(51);

                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightArm.setPower(1);
                leftArm.setPower(1);
            }

            if (gamepad2.b) {
                // Middle Level
                rightArm.setTargetPosition(90);
                leftArm.setTargetPosition(90);

                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightArm.setPower(1);
                leftArm.setPower(1);
            }

            if (gamepad2.y) {
                // Top Level
                rightArm.setTargetPosition(140);
                leftArm.setTargetPosition(140);

                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightArm.setPower(1);
                leftArm.setPower(1);
            }

            if (gamepad2.x) {
                // Coast all the way down to prepare to collect another ball/block
                rightArm.setPower(0);
                leftArm.setPower(0);

                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }



            /*
             * This block of code allows the stick to be used to adjust the position of the arm
             * even while it is running to a position. Once the desired position is reached, releasing
             * the stick will have it running to the position you left it to try and stay there.
             */
            if (gamepad2.left_stick_y != 0) {
                if (!manualControl) {
                    manualControl = true;
                    rightArm.setPower(0);
                    leftArm.setPower(0);
                    rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                leftArm.setPower(-gamepad2.left_stick_y * .5);
                rightArm.setPower(-gamepad2.left_stick_y * .5);
            } else if (manualControl) {
                manualControl = false;
                rightArm.setTargetPosition(rightArm.getCurrentPosition());
                leftArm.setTargetPosition(leftArm.getCurrentPosition());

                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightArm.setPower(1);
                leftArm.setPower(1);
            }


            // A touch sensor located where the arm bottoms out allows the encoders to zero out and
            // ensure no encoder errors build up over time
            if (!touchSensor.getState()) {
                rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Toggle code for grabbing blocks/balls
            if (gamepad2.left_bumper) {
                if (!triggerDown) {
                    triggerDown = true;
                    if (grabOpen) {
                        // Close the grabbers and turn the LEDs red
                        rightGrabber.setPosition(0);
                        leftGrabber.setPosition(1);
                        grabOpen = false;
                        setLEDs(true, false);
                    } else {
                        // Open the grabbers and turn the LEDs green
                        rightGrabber.setPosition(0.3);
                        leftGrabber.setPosition(0.7);
                        grabOpen = true;
                        setLEDs(true, true);
                    }
                }
            } else {
                triggerDown = false;
            }

            // Telemetry code for showing encoder values. Helpful during debugging.
            telemetry.addData("Left Arm Position", leftArm.getCurrentPosition());
            telemetry.addData("Right Arm Position", rightArm.getCurrentPosition());
            telemetry.addData("Distance reading", distance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }

    public void setLEDs(boolean red, boolean green) {
        // This function serves as an easy way to set all the LED indicators
        leftLEDGreen.setState(green);
        leftLEDRed.setState(red);
        rightLEDGreen.setState(green);
        rightLEDRed.setState(red);
    }
}