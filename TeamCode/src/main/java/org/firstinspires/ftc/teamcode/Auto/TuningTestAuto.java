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

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.drivetrain;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.positionalMovement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.PositionalMovementSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;


/**
 *
 */

@Autonomous(name="Tuning", group="Linear Opmode")
public class TuningTestAuto extends LinearOpMode {

    MainAuto auto = new MainAuto(autoStartSpot.RED_LEFT, this);

    Robot robot = new Robot(this, true, true);

    @Override
    public void runOpMode() {


        /*
         * TODO: Tune trackwidth
         * TODO: Fix pathing
         * TODO: Finish Auto run
         * TODO: Implement Auto switcher
         */


        robot.init(hardwareMap, 0, 0, 0, false);

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        telemetry.addData("X Loc", moving.getX());
        telemetry.addData("Y Loc", moving.getY());
        telemetry.addData("Heading", moving.getHeading());
        telemetry.update();

        waitForStart();
//
//        PositionalMovementSubsystem.turnTo(Math.PI / 2, 0.5);
//        sleep(1000);
//        PositionalMovementSubsystem.turnTo(Math.PI, 0.5);
//        sleep(1000);
//        PositionalMovementSubsystem.turnTo(0, 0.5);
//        sleep(1000);
//        PositionalMovementSubsystem.turnTo(-Math.PI / 2, 0.5);

        //Robot.moveTo(pathType.STRAIGHT, 36, 55, 0.2);
        //positionalMovement.moveToLocation(36, 55, 0.5);
        //Robot.moveToLocation(0,0,0.2);
        /*
        turnTo(PI/2, 0.5);
        sleep(1000);
        turnTo(0, 0.5);
        sleep(1000);
        turnTo((3*PI)/2, 0.5);
        sleep(1000);
        turnTo((3*PI)/2, 0.5);
        */


        //drivetrain.stop();

        boolean isAtStart = true;

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTele = dash.getTelemetry();

        while(opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            telemetry.addData("Press A to move", moving.getX());
            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.update();


            PositionalMovementSubsystem.moveTo(30,0, 0, 1, true, true, dashTele);

            /*
            if(gamepad1.a) {
                if (isAtStart) {
                    positionalMovement.moveToLocation(36, 55, 0.5);
                    isAtStart = false;
                } else {
                    positionalMovement.moveToLocation(36, 10, 0.5);
                    isAtStart = true;
                }
            }

             */

        }
    }
}
