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

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.PositionalMovementSubsystem.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.*;
import static org.firstinspires.ftc.teamcode.Subsystems.VisionPipeline.SignalColor.BLUE;
import static org.firstinspires.ftc.teamcode.Subsystems.VisionPipeline.SignalColor.GREEN;
import static org.firstinspires.ftc.teamcode.Subsystems.VisionPipeline.SignalColor.RED;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.PositionalMovementSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.VisionPipeline;


/**
 *  The autonomous for the Left side of the field.
 */

@Autonomous(name="Left Auto", group="Linear Opmode")

public class RedLeft extends LinearOpMode {

    MainAuto auto = new MainAuto(autoStartSpot.RED_LEFT, this);

    Robot robot = new Robot(this, true, true);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, 36.2, 7, 90, true);

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        telemetry.addData("X Loc", moving.getX());
        telemetry.addData("Y Loc", moving.getY());
        telemetry.addData("Heading", moving.getHeading());
        telemetry.addData("Color Guess", visionPipeline.getPredictedColor().toString());
        telemetry.addData("Hue Average", visionPipeline.getAverage());
        telemetry.update();

        VisionPipeline.SignalColor color = visionPipeline.getPredictedColor();


        while (opModeInInit()) {
            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.addData("Arm Value", armLift.getCurrentPosition());
            telemetry.addData("Color Guess", visionPipeline.getPredictedColor().toString());
            telemetry.addData("Hue Average", visionPipeline.getAverage());

            color = visionPipeline.getPredictedColor();

            telemetry.update();
        }

        waitForStart();

         pickupLeft.setPower(1);
         pickupRight.setPower(-1);

         while(coneTouch.getState()) {
             idle();
         }

        pickupLeft.setPower(0);
        pickupRight.setPower(0);


        armLift.setTargetPosition(-3760);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
        moveToLocation(36, 55, 0.5);

        double msStartPause = getRuntime();

        while (opModeIsActive() && (armLift.getCurrentPosition() > -3560)) {
            telemetry.addData("Arm Height", armLift.getCurrentPosition());
            telemetry.update();

            if(msStartPause + 3 < getRuntime()) {
                break;
            }

        }

        pickupLeft.setPower(1);
        pickupRight.setPower(-1);

        while(coneTouch.getState()) {
            idle();
        }

        pickupLeft.setPower(0.3);
        pickupRight.setPower(-0.3);

        moveToLocation(52,60,0.25);

        holOdom.updatePose();
        moving = holOdom.getPose();

        turnTo(moving.getHeading() + Math.toRadians(40), 0.5);

        pickupLeft.setPower(-1);
        pickupRight.setPower(1);
        sleep(3000);
        pickupLeft.setPower(0);
        pickupRight.setPower(0);

        moveToLocation(37, 56, -0.5, false);

        sleep(500);

        /*
        drivetrain.drive(-0.5,-0.5);


        double move = getRuntime();
        while(opModeIsActive() && move + 0.3 > getRuntime()) {
            holOdom.updatePose();
        }
        drivetrain.stop();

         */

        holOdom.updatePose();

        if(color == RED) {
            //moveToLocation(12, 60, 0.5);
            //turnTo(0, 0.5);
            drivetrain.drive(0.5, -0.5);
            sleep(500);
            drivetrain.drive(-0.5);
            sleep(900);
        } else if(color == GREEN) {
            drivetrain.drive(0.5, -0.5);
            sleep(500);
            drivetrain.drive(0.5);
            sleep(700);
        }
        drivetrain.stop();
    }
}
