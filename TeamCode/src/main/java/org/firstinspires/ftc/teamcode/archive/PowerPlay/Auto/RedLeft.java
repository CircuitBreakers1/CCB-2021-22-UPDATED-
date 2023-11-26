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

package org.firstinspires.ftc.teamcode.archive.PowerPlay.Auto;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.*;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.*;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.*;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.APRIL_SLEEVE;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot;


/**
 *  The autonomous for the Left side of the field.
 */

@Autonomous(name="Left Auto", group="Linear Opmode")
@Disabled
public class RedLeft extends LinearOpMode {


    Robot robot = new Robot(this, true, true);
    int park = 0;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, 35.8, 7, 90, APRIL_SLEEVE);

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        telemetry.addData("X Loc", moving.getX());
        telemetry.addData("Y Loc", moving.getY());
        telemetry.addData("Heading", moving.getHeading());
//        telemetry.addData("Color Guess", colorSleevePipeline.getPredictedColor().toString());
//        telemetry.addData("Hue Average", colorSleevePipeline.getAverage());
        telemetry.addData("Location Guess", apriltagSleevePipeline.getPrediction());
        try {
            telemetry.addData("Tag ID", apriltagSleevePipeline.getLatestDetections().get(0).id);
        } catch (Exception e) {
            telemetry.addData("Tag ID", "No tag detected");
        }
        try {
            park = apriltagSleevePipeline.getPrediction().getValue();
        } catch (Exception ignored) {

        }
        telemetry.update();

//        ColorSleevePipeline.SignalColor color = colorSleevePipeline.getPredictedColor();


        while (opModeInInit()) {
            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.addData("Arm Value", armLift.getCurrentPosition());
//            telemetry.addData("Color Guess", colorSleevePipeline.getPredictedColor().toString());
//            telemetry.addData("Hue Average", colorSleevePipeline.getAverage());

//            color = colorSleevePipeline.getPredictedColor();
//
//            if(color == RED) {
//                pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
//            } else if (color == BLUE) {
//                pattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
//            } else if (color == GREEN) {
//                pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
//            }

            telemetry.addData("Location Guess", apriltagSleevePipeline.getPrediction());
            try {
                telemetry.addData("Tag ID", apriltagSleevePipeline.getLatestDetections().get(0).id);
            } catch (Exception e) {
                telemetry.addData("Tag ID", "No tag detected");
            }
            try {
                park = apriltagSleevePipeline.getPrediction().getValue();
            } catch (Exception ignored) {

            }


//            blinkinLedDriver.setPattern(pattern);

            telemetry.update();
        }

        waitForStart();

        double startTime = getRuntime();

        pickupLeft.setPower(1);
        pickupRight.setPower(-1);

        while(coneTouch.getState() && getRuntime() - startTime < 0.5) {
        }

//        pickupLeft.setPower(0.2);
//        pickupRight.setPower(-0.2);

        pickupLeft.setPower(0.1);
        pickupRight.setPower(-0.1);

        LiftSubsystem.setTarget(Medium);



        //Move to the medium junction and turn
        moveTo(36,47, 0.8, false);

        turn(0,0.8);

        //Move into the junction, drop and move out
        //Dropping
        moveTo(39.5, 47, 0.6);
        pickupLeft.setPower(-1);
        pickupRight.setPower(1);
        sleep(1500);
        pickupLeft.setPower(0);
        pickupRight.setPower(0);
        moveTo(36, 47, 0.7, false);

        //Move to the center of the cycle path and turn
        moveTo(36, 59.5, 0.7, false);
        turnTo180(0.5);

        LiftSubsystem.setTarget(ConeStack);
        //moveTo(16, 60, 0.5);


//
//        moveTo(11.2, 60, 0.5);
//        pickupLeft.setPower(1);
//        pickupRight.setPower(-1);
//        LiftSubsystem.setPosition(coneLevel[5 - conesInStack]);
//
//        while(coneTouch.getState() || !LiftSubsystem.isAtTarget()) {
//            LiftSubsystem.updatePositional();
//        }

        /*
        //Cycle timing information
        //TODO: Tune this
        int conesInStack = 5;
        int[] coneLevel = {200, 140, 100};
        double pickupTime = 2.5; //Time to lineup and pickup cone when starting in the lined square
        double movementTime = 4.5; //Time to move from the lined square to location of the junction
        double dropTime = 0.5; //Time to line up and drop the cone
        double[] parkFromLine = {0.5, 2, 2.25}; //Time to park from lined square
        double[] parkFromJunction = {2.25, 1.75, 1}; //Time to park from junction

        //Begin the cycle loop
        while(opModeIsActive()) {
            //Set arm height and go to cone stack
            LiftSubsystem.setTarget(ConeStack);
            //moveTo(16, 60, 0.5);

            //Check if there is time to pickup cone, otherwise park
            if(getRemainingTime(startTime, getRuntime()) < pickupTime + parkFromLine[park]) {
                break;
            }

            moveTo(24, 60, 0.8, false);

            //Pickup the cone
            if(conesInStack == 5) {
                moveTo(10.5, 59.25, 0.4);
            } else {
                moveTo(9.25, 60.25, 0.4);
            }
            pickupLeft.setPower(1);
            pickupRight.setPower(-1);
            LiftSubsystem.setPosition(coneLevel[5 - conesInStack]);

            while(coneTouch.getState() && !LiftSubsystem.isAtTarget()) {
                LiftSubsystem.updatePositional();
            }

            conesInStack--;

            pickupLeft.setPower(0.1);
            pickupRight.setPower(-0.1);

            LiftSubsystem.setTarget(High);

            while(opModeIsActive() && armLift.getCurrentPosition() < 700) {

            }
//
//            while(armLift.getCurrentPosition() < Intake.getPosition()) {
//                LiftSubsystem.updatePositional();
//            }
//
//            moveTo(16, 60, 0.5);

//            LiftSubsystem.setTarget(Min);

            //Check if there is time to drop off cone, otherwise park
            if(getRemainingTime(startTime, getRuntime()) < dropTime + movementTime + parkFromJunction[park]) {
                break;
            }

            //Drop off the cone
            moveTo(47.5, 58.25, 0.7);
            turn(90, 0.7);
            moveTo(47.5, 61.25, 0.7);
            pickupLeft.setPower(-1);
            pickupRight.setPower(1);
            double startDrop = getRuntime();
            while (getRuntime() - startDrop < 1.5) {
                LiftSubsystem.updatePositional();
            }
            pickupLeft.setPower(0);
            pickupRight.setPower(0);
            moveTo(48, 60, 0.7, false);

            //Check if there is still time to cycle, otherwise park
            if(getRemainingTime(startTime, getRuntime()) < movementTime + pickupTime + parkFromLine[park]) {
                break;
            }

            //Make sure there is still a cone in the stack
            if(conesInStack <= 0) {
                break;
            }

            turnTo180(0.7);
        }

         */

        //sleep(500);

        moveTo(35,58,0.7);

        LiftSubsystem.setTarget(Min);

        //Park in designated spot
        holOdom.updatePose();
        moving = holOdom.getPose();
        double x = moving.getX(), y = moving.getY();

        LiftSubsystem.setTarget(Min);
        switch (park) {
            case 0:
                moveTo(14.5, 60, 0.6);
                break;
            case 1:
                moveTo(35, 58, 0.7);
                break;
            case 2:
                moveTo(60, 56, 0.7);
                break;
        }

        //turn(90, 0.75);

        //Allow the arm to stabilize
        while (opModeIsActive()) {
            LiftSubsystem.updatePositional();
            if (armLift.getCurrentPosition() < 10) {
                break;
            }
        }
    }
}
