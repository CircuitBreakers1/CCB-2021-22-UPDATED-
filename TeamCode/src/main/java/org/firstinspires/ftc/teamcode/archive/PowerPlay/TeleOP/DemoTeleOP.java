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

package org.firstinspires.ftc.teamcode.archive.PowerPlay.TeleOP;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.High;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Intake;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Low;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Medium;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Min;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.armLift;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.backOdo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.blinkinLedDriver;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.NO_CAM;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.coneTouch;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.holOdom;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.imu;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.leftOdo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.pattern;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.pickupLeft;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.pickupRight;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.rightFront;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.rightOdo;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.TeleOPTargetingSubsystem;

@TeleOp(name = "Demo TeleOP")
public class DemoTeleOP extends LinearOpMode {
    Robot robot = new Robot(this, false, true);
    Orientation angles;
    boolean isIntaking = false;
    boolean isOutputting = false;
    boolean manualControl = false;
    boolean canDrop = false;
    int[] armLimits = {-2000, -2600, -3560};


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, 35.8, 7, 90, NO_CAM);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //initTargetingSubsystem(holOdom);

        //Set the first gamepad led to purple, and the second to yellow
        gamepad1.setLedColor(255, 0, 255, -1);
        gamepad2.setLedColor(255, 255, 0, -1);

        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;

        blinkinLedDriver.setPattern(pattern);

        //Prepare the rumble patterns for driver 2
        RumbleEffect dropAllowed = new RumbleEffect.Builder().addStep(1,1, 100).addStep(0,0,50).addStep(1,1,100).build();
        RumbleEffect dropNotAllowed = new RumbleEffect.Builder().addStep(0.5,0.5, 150).build();
        RumbleEffect leftSideButtonPress = new RumbleEffect.Builder().addStep(0.5,0, 750).build();
        RumbleEffect rightSideButtonPress = new RumbleEffect.Builder().addStep(0,0.5, 750).build();

        waitForStart();

        //Orange
        gamepad1.setLedColor(255, 165, 0, -1);
        gamepad2.setLedColor(255, 165, 0, -1);


        while (opModeIsActive()) {
            holOdom.updatePose();
            Pose2d currentLocation = holOdom.getPose();
            double angle = currentLocation.getHeading();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            telemetry.addData("Target Junction", "X: " + TeleOPTargetingSubsystem.targetJunctions[0] + " Y: " + TeleOPTargetingSubsystem.targetJunctions[1]);
            telemetry.addData("Target Drop Spot", "X: " + TeleOPTargetingSubsystem.targetPosition[0] + " Y: " + TeleOPTargetingSubsystem.targetPosition[1]);
            telemetry.addData("Distance to Drop", TeleOPTargetingSubsystem.distance);
            telemetry.addData("Arm Position", armLift.getCurrentPosition());
            telemetry.addData("Arm Target" , armLift.getTargetPosition());
            telemetry.addData("Robot X", currentLocation.getX());
            telemetry.addData("Robot Y", currentLocation.getY());
            telemetry.addData("Robot Angle", angle);
            telemetry.addData("Gyro Angle", angles.firstAngle);
            telemetry.addData("Angle Error", Math.abs(angles.firstAngle) - Math.abs(angle));

            telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            telemetry.addData("Back Odo", backOdo.getCurrentPosition());
            telemetry.addData("Arm Spot", armLift.getCurrentPosition());
            telemetry.addData("Left Back", leftBack.getCurrentPosition());
            telemetry.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            Pose2d moving = holOdom.getPose();
            double heading = moving.getHeading();

            double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
            double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

            leftFront.setPower(0.45 * (y + x + rx));
            leftBack.setPower(0.45 * (y - x + rx));
            rightFront.setPower(0.45 * (y - x - rx));
            rightBack.setPower(0.45 * (y + x - rx));

            if(gamepad2.dpad_up) {
                pickupLeft.setPower(1);
                pickupRight.setPower(-1);
                LiftSubsystem.setTarget(Min);
                isIntaking = true;
                isOutputting = false;
            }
            if(!isIntaking && gamepad2.left_bumper) {
                pickupLeft.setPower(1);
                pickupRight.setPower(-1);
                isIntaking = true;
                isOutputting = false;
            }
            if(!isOutputting && gamepad2.dpad_down) {
                pickupLeft.setPower(-1);
                pickupRight.setPower(1);
                isOutputting = true;
                isIntaking = false;
            }
            if(gamepad2.dpad_left || (!coneTouch.getState() && isIntaking)) {
                pickupLeft.setPower(0);
                pickupRight.setPower(0);
                isIntaking = false;
                isOutputting = false;
            }

            if(gamepad2.a) {
                LiftSubsystem.setTarget(Low);
            } else if (gamepad2.b) {
                LiftSubsystem.setTarget(Medium);
            } else if (gamepad2.x) {
                LiftSubsystem.setTarget(Intake);
            } else if (gamepad2.y) {
                LiftSubsystem.setTarget(High);
            } else if (gamepad2.right_bumper) {
                LiftSubsystem.setTarget(Min);
            }

            //LiftSubsystem.manualControl(-gamepad2.right_stick_y);

            LiftSubsystem.updatePositional();

//            if(colorJunctionLeftPipeline.isJunctionDetected() && colorJunctionRightPipeline.isJunctionDetected()) {
//                if(!canDrop) {
//                    canDrop = true;
//                    gamepad1.setLedColor(0, 255, 0, -1);
//                    gamepad2.setLedColor(0,255,0, -1);
//                    gamepad2.runRumbleEffect(dropAllowed);
//                    gamepad1.runRumbleEffect(dropAllowed);
//                }
//            } else if (canDrop) {
//                canDrop = false;
//                gamepad1.setLedColor(255, 165, 0, -1);
//                gamepad2.setLedColor(255, 165, 0, -1);
//                gamepad2.runRumbleEffect(dropNotAllowed);
//                gamepad1.runRumbleEffect(dropNotAllowed);
//            }
        }
    }
}
