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


/**
 * This file contains test code for making encoder movement functions.
 */



@TeleOp(name="Encoder Test", group="Linear Opmode")
public class TestEncoders extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    public static final int PulsesPerRot = 537;
    public static final double wheelDiameterMM = 96;
    public static final double wheelDiameterIN = wheelDiameterMM / 25.4;
    public static final double wheelCircumferenceIN = 3.14 * wheelDiameterIN;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.dcMotor.get("leftFront");
        rightFront  = hardwareMap.dcMotor.get("rightFront");
        leftBack  = hardwareMap.dcMotor.get("leftBack");
        rightBack  = hardwareMap.dcMotor.get("rightBack");
        
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        driveIN(12, 0.5);

        sleep(1000);

    }

    public void stopDrive() {
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    public void startDrive(double power) {
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
    }

    public void driveRots(double rots, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticks = Math.round(rots * PulsesPerRot);

        leftFront.setTargetPosition((int) ticks);
        rightFront.setTargetPosition((int) ticks);
        leftBack.setTargetPosition((int) ticks);
        rightBack.setTargetPosition((int) ticks);

        startDrive(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && opModeIsActive()) {
            /**
             * Robot gets some free time.
             * What does it do during it's free time?
             * No one knows.
             * Maybe it's one of those chess bots
             * or perhaps it's training a neural network
             * through a harsh yet effective process of generational evolution.
             * It probably just naps. It works pretty hard.
             * Even though we might not know what it does, all that matters to us is that it comes
             * back when the motors are done.
             */
        }

        stopDrive();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveIN(double inches, double power) {
        double rotations = (inches / wheelCircumferenceIN);
        driveRots(rotations, power);
    }
}
