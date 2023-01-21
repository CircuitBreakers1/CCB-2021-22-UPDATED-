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

package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.cameraInit.NO_CAM;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.drivetrain;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.ArrayList;

@Autonomous(name = "Motor Speed Tuning", group = "Tuning")
public class MotorSpeedTuning extends LinearOpMode {
    Robot robot = new Robot(this, false, false);

    double acceptableErrorPercent = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, 0, 0, 0, NO_CAM);

        waitForStart();

        //Create doubles for the current motor feedforward values
        double currentLFFF = 1;
        double currentRFFF = 1;
        double currentLBFF = 1;
        double currentRBFF = 1;

        telemetry.addData("Status", "Preparing Test");

        telemetry.addData("Left Front Feedforward", currentLFFF);
        telemetry.addData("Right Front Feedforward", currentRFFF);
        telemetry.addData("Left Back Feedforward", currentLBFF);
        telemetry.addData("Right Back Feedforward", currentRBFF);
        telemetry.update();

        drivetrain.resetEncoders();

        //Create an arrayList to store each motor feedforward values
        ArrayList<Double> leftFrontFeedforward = new ArrayList<>();
        ArrayList<Double> rightFrontFeedforward = new ArrayList<>();
        ArrayList<Double> leftBackFeedforward = new ArrayList<>();
        ArrayList<Double> rightBackFeedforward = new ArrayList<>();

        //Set the motors zero power mode to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            //Add telemetry for the current motor feedforward values and the current status


            //Reset the encoders
            drivetrain.resetEncoders();

            sleep(2000);

            //Run the motors for 5 seconds at full speed
            leftBack.setPower(1 * currentLBFF);
            leftFront.setPower(1 * currentLFFF);
            rightFront.setPower(1 * currentRFFF);
            rightBack.setPower(1 * currentRBFF);

            double curTime = getRuntime();
            //Update the telemetry to show the current status for 5 seconds
            while (opModeIsActive() && getRuntime() < curTime + 5) {
                telemetry.addData("Left Front Feedforward", currentLFFF);
                telemetry.addData("Right Front Feedforward", currentRFFF);
                telemetry.addData("Left Back Feedforward", currentLBFF);
                telemetry.addData("Right Back Feedforward", currentRBFF);
                telemetry.addData("Status", "Running Test");
                telemetry.update();
            }

            double testTime = getRuntime() - curTime;

            //Stop the motors
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            //Get the encoder values
            int leftBackEncoder = leftBack.getCurrentPosition();
            int leftFrontEncoder = leftBack.getCurrentPosition();
            int rightFrontEncoder = rightFront.getCurrentPosition();
            int rightBackEncoder = rightBack.getCurrentPosition();

            //Calculate the ticks/second for each motor
            double leftBackTicks = leftBackEncoder / testTime;
            double leftFrontTicks = leftFrontEncoder / testTime;
            double rightFrontTicks = rightFrontEncoder / testTime;
            double rightBackTicks = rightBackEncoder / testTime;



            //Get the smallest ticks/second value
            double minTicks = Math.min(Math.min(leftBackTicks, leftFrontTicks), Math.min(rightFrontTicks, rightBackTicks));

            //Store the motor feedforward values in their lists
            leftFrontFeedforward.add(currentLFFF);
            rightFrontFeedforward.add(currentRFFF);
            leftBackFeedforward.add(currentLBFF);
            rightBackFeedforward.add(currentRBFF);


            //Calculate the feedforward value for each motor
            double lfAdjust = minTicks / leftBackTicks;
            double rfAdjust = minTicks / rightFrontTicks;
            double lbAdjust = minTicks / leftBackTicks;
            double rbAdjust = minTicks / rightBackTicks;

            //If all of the variance is within the acceptable error percent, stop the test
            if (Math.abs(lfAdjust - 1) < acceptableErrorPercent && Math.abs(rfAdjust - 1) < acceptableErrorPercent && Math.abs(lbAdjust - 1) < acceptableErrorPercent && Math.abs(rbAdjust - 1) < acceptableErrorPercent) {
                telemetry.addData("Status", "Test Complete");

                telemetry.addData("Left Front Feedforward", currentLFFF);
                telemetry.addData("Right Front Feedforward", currentRFFF);
                telemetry.addData("Left Back Feedforward", currentLBFF);
                telemetry.addData("Right Back Feedforward", currentRBFF);

                telemetry.addData("Left Front Ticks", leftFrontTicks);
                telemetry.addData("Right Front Ticks", rightFrontTicks);
                telemetry.addData("Left Back Ticks", leftBackTicks);
                telemetry.addData("Right Back Ticks", rightBackTicks);

                telemetry.addData("Left Front Variance", lfAdjust);
                telemetry.addData("Right Front Variance", rfAdjust);
                telemetry.addData("Left Back Variance", lbAdjust);
                telemetry.addData("Right Back Variance", rbAdjust);

                telemetry.update();
                break;
            }

            //Update the current motor feedforward values using the variance
            currentLFFF *= lfAdjust;
            currentRFFF *= rfAdjust;
            currentLBFF *= lbAdjust;
            currentRBFF *= rbAdjust;

            telemetry.addData("Status", "Tolerance Not Reached, Rerunning Test");

            telemetry.addData("Left Front Feedforward", currentLFFF);
            telemetry.addData("Right Front Feedforward", currentRFFF);
            telemetry.addData("Left Back Feedforward", currentLBFF);
            telemetry.addData("Right Back Feedforward", currentRBFF);

            telemetry.addData("Left Front Ticks", leftFrontTicks);
            telemetry.addData("Right Front Ticks", rightFrontTicks);
            telemetry.addData("Left Back Ticks", leftBackTicks);
            telemetry.addData("Right Back Ticks", rightBackTicks);

            telemetry.addData("Left Front Variance", lfAdjust);
            telemetry.addData("Right Front Variance", rfAdjust);
            telemetry.addData("Left Back Variance", lbAdjust);
            telemetry.addData("Right Back Variance", rbAdjust);
            telemetry.update();
        }

        while (opModeIsActive()) {
            idle();
        }
    }
}
