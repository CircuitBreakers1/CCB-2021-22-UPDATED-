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

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
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
 *
 * Notes:
 * Grabbing Servos:
 * Open: LEDs Amber
 * Block Detected: LEDs Green
 * Closed: LEDs Red
 */

@TeleOp(name = "TeleOP", group = "Linear Opmode")
public class TeleOP extends LinearOpMode {

    HardwareInit robot = new HardwareInit();

    /**
     * Array for smoothing distance inputs. The smoothed value is stored at index <b>6</b>
     */
    double distance[] = new double[7];

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //Various booleans used for toggle buttons
        boolean spinOn = false;
        boolean aDown = false;

        boolean grabOpen = true;
        boolean triggerDown = false;

        boolean manualControl = false;

        double lastCheck = 0;

        // Left 1 is down, right 0 is down
        // Start Open
        robot.rightGrabber.setPosition(0.3);
        robot.leftGrabber.setPosition(0.7);
        // Set the LEDs green to show the grabbing servos are open
        setLEDs(true, true);


        waitForStart();

        while (opModeIsActive()) {
            if(System.currentTimeMillis() > lastCheck + 166) {
                averageVal(robot.distance, 0.5);
            }

            // Simple tank drive
            robot.leftFront.setPower(-gamepad1.left_stick_y);
            robot.leftBack.setPower(-gamepad1.left_stick_y);
            robot.rightBack.setPower(-gamepad1.right_stick_y);
            robot.rightFront.setPower(-gamepad1.right_stick_y);


            /*
            leftArm.setPower(-gamepad2.left_stick_y * .5);
            rightArm.setPower(-gamepad2.left_stick_y * .5);
            */

            if(distance[6] < 1.5 && grabOpen) {
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
            if (gamepad2.right_bumper) {
                if (!aDown) {
                    aDown = true;
                    if (spinOn) {
                        robot.backSpinner.setPower(0);
                        robot.backSpinner2.setPower(0);
                        spinOn = false;
                    } else {
                        robot.backSpinner.setPower(-0.65);
                        robot.backSpinner2.setPower(-0.65);
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
                robot.rightArm.setTargetPosition(51);
                robot.leftArm.setTargetPosition(51);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
            }

            if (gamepad2.b) {
                // Middle Level
                robot.rightArm.setTargetPosition(90);
                robot.leftArm.setTargetPosition(90);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
            }

            if (gamepad2.y) {
                // Top Level
                robot.rightArm.setTargetPosition(140);
                robot.leftArm.setTargetPosition(140);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
            }

            if (gamepad2.x) {
                // Coast all the way down to prepare to collect another ball/block
                robot.rightArm.setPower(0);
                robot.leftArm.setPower(0);

                robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            // Toggle code for grabbing blocks/balls
            if (gamepad2.left_bumper) {
                if (!triggerDown) {
                    triggerDown = true;
                    if (grabOpen) {
                        // Close the grabbers and turn the LEDs red
                        robot.rightGrabber.setPosition(0);
                        robot.leftGrabber.setPosition(1);
                        grabOpen = false;
                        setLEDs(true, false);
                    } else {
                        // Open the grabbers and turn the LEDs green
                        robot.rightGrabber.setPosition(0.3);
                        robot.leftGrabber.setPosition(0.7);
                        grabOpen = true;
                        setLEDs(true, true);
                    }
                }
            } else {
                triggerDown = false;
            }

            // Telemetry code for showing encoder values. Helpful during debugging.
            telemetry.addData("Left Arm Position", robot.leftArm.getCurrentPosition());
            telemetry.addData("Right Arm Position", robot.rightArm.getCurrentPosition());
            telemetry.addData("Distance reading", distance[6]) ;
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

    /**
     * This function is ran to attempt to make the values of the distance sensor smoother. This function should only be ran at max every 100 millis,
     * which will store the last second of values. The value is stored in distance[6] and all other
     * stored values can be checked in the program, including the most recent check, which can reduce
     * processing on pulling the distance
     * @param distanceSensor Distance sensor to use. Throws an error if null for obvious reasons
     * @param discardDifference If a value is further away from the most recent input than this number
     *                          in inches it is discarded, and all other values after it are discarded too.
     */
    public void averageVal(@NonNull DistanceSensor distanceSensor, double discardDifference) {
        double mostRecent = distanceSensor.getDistance(DistanceUnit.INCH);
        double avgHelp = 0;
        double avgCount = 0;
        boolean discardRest = false;
        //Shift the array
        for (int i = distance.length - 2; i >= 0; i--){
            distance[i + 1] = distance[i];
        }

        //Add the new value
        distance[0] = mostRecent;

        //Check values to throw out
        for (int i = 1; i < distance.length - 1; i++) {
            if ((Math.abs(mostRecent - distance[i]) > discardDifference) || discardRest) {
                distance[i] = 0;
                discardRest = true;
            }
        }

        //Calcuate and store the average
        for (int i = 0; i < distance.length - 1; i++) {
            if (distance[i] == 0) {
                break;
            }
            avgHelp = avgHelp + distance[i];
            avgCount++;
        }
        distance[6] = avgHelp / avgCount;
    }
}