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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class is used to initialize the robot hardware in multiple programs and be the only spot it
 * needs to be changed
 */
public class HardwareInit
{
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor backSpinner;
    public DcMotor intake;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public DigitalChannel touchSensor;
    public DigitalChannel cargoTouch;
    public DigitalChannel leftLEDGreen;
    public DigitalChannel leftLEDRed;
    public DigitalChannel rightLEDGreen;
    public DigitalChannel rightLEDRed;
    public DistanceSensor distance;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareInit(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        backSpinner  = hwMap.get(DcMotor.class, "backSpinner");
        intake  = hwMap.get(DcMotor.class, "intake");
        leftArm  = hwMap.get(DcMotor.class, "leftArm");
        rightArm  = hwMap.get(DcMotor.class, "rightArm");

        // Set motor direction
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);


        // Sets the zero power behavior to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        // Define and initialize ALL installed servos.
        leftGrabber  = hwMap.get(Servo.class, "leftGrabber");
        rightGrabber = hwMap.get(Servo.class, "rightGrabber");

        // Define and initialize all digital devices
        distance = hwMap.get(DistanceSensor.class, "distance");
        touchSensor = hwMap.get(DigitalChannel.class, "touchSensor");
        cargoTouch = hwMap.get(DigitalChannel.class, "cargoTouch");
        leftLEDGreen = hwMap.get(DigitalChannel.class, "leftLEDGreen");
        rightLEDGreen = hwMap.get(DigitalChannel.class, "rightLEDGreen");
        leftLEDRed = hwMap.get(DigitalChannel.class, "leftLEDRed");
        rightLEDRed = hwMap.get(DigitalChannel.class, "rightLEDRed");

        // The digital channel defaults to inputs, so we have to set the LEDs channels to outputs
        leftLEDRed.setMode(DigitalChannel.Mode.OUTPUT);
        leftLEDGreen.setMode(DigitalChannel.Mode.OUTPUT);
        rightLEDRed.setMode(DigitalChannel.Mode.OUTPUT);
        rightLEDGreen.setMode(DigitalChannel.Mode.OUTPUT);
    }
}


