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

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.*;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;


/**
 *
 */

@Autonomous(name="Location Test Auto", group="Linear Opmode")

public class RedLeftTest extends LinearOpMode {

    MainAuto auto = new MainAuto(autoStartSpot.RED_LEFT, this);

    Robot robot = new Robot(this, true, true);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, 36.2, 7, 90, false);

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        telemetry.addData("X Loc", moving.getX());
        telemetry.addData("Y Loc", moving.getY());
        telemetry.addData("Heading", moving.getHeading());
        telemetry.update();

        while (opModeInInit()) {
            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.addData("Arm Value", armLift.getCurrentPosition());
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


        armLift.setTargetPosition(-3560);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
        positionalMovement.moveToLocation(36, 55, 0.5);
        positionalMovement.moveToLocation(53,61,0.25);


        pickupLeft.setPower(-1);
        pickupRight.setPower(1);
        sleep(3000);
        pickupLeft.setPower(0);
        pickupRight.setPower(0);



        drivetrain.stop();

        while(opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.addData("Arm Value", armLift.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }
}
