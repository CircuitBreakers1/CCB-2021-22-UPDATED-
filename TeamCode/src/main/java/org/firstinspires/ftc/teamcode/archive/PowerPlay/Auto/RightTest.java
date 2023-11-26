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

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.getRemainingTime;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.moveTo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.moveToLocation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 *  The autonomous for the Right Side of the Field
 */

@Autonomous(name="Right Auto", group="Linear Opmode")
@Disabled
public class RightTest extends LinearOpMode {


    MainAuto mainAuto = new MainAuto(autoStartSpot.RIGHT, this);

    //Robot robot = new Robot(this, true, true);

    @Override
    public void runOpMode() {
        mainAuto.initAuto();
        waitForStart();
        mainAuto.runAuto();
    }
//
//
//        robot.init(hardwareMap, 35.8, 7, 90, true);
//
//        holOdom.updatePose();
//        Pose2d moving = holOdom.getPose();
//
//        telemetry.addData("X Loc", moving.getX());
//        telemetry.addData("Y Loc", moving.getY());
//        telemetry.addData("Heading", moving.getHeading());
//        telemetry.addData("Color Guess", visionPipeline.getPredictedColor().toString());
//        telemetry.addData("Hue Average", visionPipeline.getAverage());
//        telemetry.update();
//
//        VisionPipeline.SignalColor color = visionPipeline.getPredictedColor();
//
//
//        while (opModeInInit()) {
//            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
//                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            telemetry.addData("X Loc", moving.getX());
//            telemetry.addData("Y Loc", moving.getY());
//            telemetry.addData("Heading", moving.getHeading());
//            telemetry.addData("Arm Value", armLift.getCurrentPosition());
//            telemetry.addData("Color Guess", visionPipeline.getPredictedColor().toString());
//            telemetry.addData("Hue Average", visionPipeline.getAverage());
//
//            color = visionPipeline.getPredictedColor();
//
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        double startTime = getRuntime();
//
//        pickupLeft.setPower(1);
//        pickupRight.setPower(-1);
//
//        while(coneTouch.getState() && getRuntime() - startTime < 2) {
//        }
//
////        pickupLeft.setPower(0.2);
////        pickupRight.setPower(-0.2);
//
//        pickupLeft.setPower(0.1);
//        pickupRight.setPower(-0.1);
//
//        LiftSubsystem.setTarget(Medium);
//
//
//
//        //Move to the medium junction and turn
//        moveTo(36,47, 0.6, false);
//
//        turnTo180(0.5);
//
//        //Move into the junction, drop and move out
//        moveTo(32.75, 47, 0.6);
//        pickupLeft.setPower(-1);
//        pickupRight.setPower(1);
//        sleep(1500);
//        pickupLeft.setPower(0);
//        pickupRight.setPower(0);
//        moveTo(36, 47, 0.6, false);
//
//        //Move to the center of the cycle path and turn
//        moveTo(36, 59.5, 0.6, false);
//        turn(0, 0.5);
//
//        LiftSubsystem.setTarget(ConeStack);
//        //moveTo(16, 60, 0.5);
//
//
////
////        moveTo(11.2, 60, 0.5);
////        pickupLeft.setPower(1);
////        pickupRight.setPower(-1);
////        LiftSubsystem.setPosition(coneLevel[5 - conesInStack]);
////
////        while(coneTouch.getState() || !LiftSubsystem.isAtTarget()) {
////            LiftSubsystem.updatePositional();
////        }
//
//        //Cycle timing information
//        //TODO: Tune this
//        int conesInStack = 5;
//        int[] coneLevel = {300, 220, 140};
//        double pickupTime = 2.5; //Time to lineup and pickup cone when starting in the lined square
//        double movementTime = 2; //Time to move from the lined square to location of the junction
//        double dropTime = 0.5; //Time to line up and drop the cone
//        double[] parkFromLine = {2.5, 2, 1.5}; //Time to park from lined square
//        double[] parkFromJunction = {1, 1.5, 2}; //Time to park from junction
//
//        //Begin the cycle loop
//        while(opModeIsActive()) {
//            //Set arm height and go to cone stack
//            LiftSubsystem.setTarget(ConeStack);
//            //moveTo(16, 60, 0.5);
//
//            //Check if there is time to pickup cone, otherwise park
//            if(getRemainingTime(startTime, getRuntime()) < pickupTime + parkFromLine[color.getValue()]) {
//                break;
//            }
//
//            //Pickup the cone
//            moveTo(62.25, 59.5, 0.6);
//            pickupLeft.setPower(1);
//            pickupRight.setPower(-1);
//            LiftSubsystem.setPosition(coneLevel[5 - conesInStack]);
//
//            while(coneTouch.getState() && !LiftSubsystem.isAtTarget()) {
//                LiftSubsystem.updatePositional();
//            }
//
//            conesInStack--;
//
//            pickupLeft.setPower(0.1);
//            pickupRight.setPower(-0.1);
//
//            LiftSubsystem.setTarget(High);
//
//            while(opModeIsActive() && armLift.getCurrentPosition() < 700) {
//
//            }
//
//            //Check if there is time to drop off cone, otherwise park
//            if(getRemainingTime(startTime, getRuntime()) < dropTime + movementTime + parkFromJunction[color.getValue()]) {
//                break;
//            }
//
//            //Drop off the cone
//            moveTo(24.5, 59, 0.6);
//            turn(90, 0.5);
//            moveTo(23.75, 61.5, 0.6);
//            pickupLeft.setPower(-1);
//            pickupRight.setPower(1);
//            double startDrop = getRuntime();
//            while (getRuntime() - startDrop < 1.5) {
//                LiftSubsystem.updatePositional();
//            }
//            pickupLeft.setPower(0);
//            pickupRight.setPower(0);
//            moveTo(24, 60, 0.6, false);
//
//            if(true) {
//                break;
//            }
//
//            //Check if there is still time to cycle, otherwise park
//            if(getRemainingTime(startTime, getRuntime()) < movementTime + pickupTime + parkFromLine[color.getValue()]) {
//                break;
//            }
//
//            //Make sure there is still a cone in the stack
//            if(conesInStack <= 0) {
//                break;
//            }
//
//            turn(0,0.5);
//        }
//
//        LiftSubsystem.setTarget(Min);
//
//        //Park in designated spot
//        holOdom.updatePose();
//        moving = holOdom.getPose();
//        double x = moving.getX(), y = moving.getY();
//
//        LiftSubsystem.setTarget(Min);
//        switch (color) {
//            case RED:
//                moveTo(12, 58, 0.7);
//                break;
//            case BLUE:
//                moveTo(36, 58, 0.7);
//                break;
//            case GREEN:
//                moveTo(60, 56, 0.7);
//                break;
//        }
//
//        //turn(0, 0.75);
//
//        //Allow the arm to stabilize
//        while (opModeIsActive()) {
//            LiftSubsystem.updatePositional();
//            if (armLift.getCurrentPosition() < 10) {
//                break;
//            }
//        }
//    }
}
