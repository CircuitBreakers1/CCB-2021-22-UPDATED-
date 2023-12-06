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

package org.firstinspires.ftc.teamcode.TeleOP;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.*;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.endThetaPI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.endX;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.endY;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.startThetaPI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.startX;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.startY;

import static java.lang.Math.PI;
import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PoseSupply;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

/**
 * Demonstrates new features
 */
@TeleOp(name = "Odo Test TeleOP", group = "")
public class OdoTestTeleOp extends LinearOpMode {
    Robot2023 robot = new Robot2023();
    double aprilX = 0;
    double aprilY = 0;
    double aprilHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true, this);

        telemetry.addData("Status", "Initialized");
        holOdom.updatePose(new Pose2d(startX, startY, new Rotation2d(startThetaPI * PI)));
        Pose2d pose = holOdom.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", pose.getHeading());

        while (opModeInInit()) {
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            robot.holoDrivetrain.smoothDrive(x, y, -gamepad1.right_stick_x);
            holOdom.updatePose();
            telemetry.addData("Left Odo", leftFront.getCurrentPosition());
            telemetry.addData("Right Odo", leftBack.getCurrentPosition() * -1);
            telemetry.addData("Back Odo", rightFront.getCurrentPosition());
            telemetry.addData("X", holOdom.getPose().getX());
            telemetry.addData("Y", holOdom.getPose().getY());
            telemetry.addData("Heading", holOdom.getPose().getHeading());
            telemetry.update();
//
        }

        waitForStart();
//
//        robot.movementSubsystem.moveToPose(16,0,0,1);
//        robot.movementSubsystem.moveToPose(16,16,0,1);
//        robot.movementSubsystem.moveToPose(0,16,PI,1);
//        robot.movementSubsystem.moveToPose(0,0, PI * 0.25, 1);

        robot.movementSubsystem.moveTo(PoseSupply.APRILTAG_BLUE_MIDDLE, endX, endY, endThetaPI * PI, 1);
    }

//    @Override
//    public void init() {
////        robot.init(hardwareMap, true);
//
//    }
//
//    @Override
//    public void init_loop() {
//    }
//
//    @Override
//    public void start() {
//
//    }
//
//    @Override
//    public void loop() {
////        double x = -gamepad1.left_stick_x;
////        double y = gamepad1.left_stick_y;
////        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
////        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
////        robot.holoDrivetrain.smoothDrive(rotX, rotY, -gamepad1.right_stick_x);
////        holOdom.updatePose();
////        Pose2d aprilPose = robot.getPoseFromAprilTag();
////        if (aprilPose != null) {
////            aprilX = aprilPose.getX();
////            aprilY = aprilPose.getY();
////            aprilHeading = aprilPose.getHeading();
////        }
////        telemetry.addData("April X", aprilX);
////        telemetry.addData("April Y", aprilY);
////        telemetry.addData("April Heading", aprilHeading);
////        telemetry.addData("Left Odo", leftFront.getCurrentPosition());
////        telemetry.addData("Right Odo", leftBack.getCurrentPosition() * -1);
////        telemetry.addData("Back Odo", rightFront.getCurrentPosition());
////        telemetry.addData("X", holOdom.getPose().getX());
////        telemetry.addData("Y", holOdom.getPose().getY());
////        telemetry.addData("Heading", holOdom.getPose().getHeading());
////        telemetry.addData("Left Front", leftFront.get());
////        telemetry.addData("Right Front", rightFront.get());
////        telemetry.addData("Left Back", leftBack.get());
////        telemetry.addData("Right Back", rightBack.get());


}
