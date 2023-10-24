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

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.imu;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.intake;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.lift;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.liftRaise;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.slidePush;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.wrist;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmState;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

/**
 * Demonstrates new features
 */
@TeleOp(name = "TeleOP", group = "")
public class LastJohnTeleOp extends OpMode {

    Robot2023 robot = new Robot2023();
    boolean toggleStart = false, autoWrist = false;
    boolean toggleIntake = false, toggleDown = false;


    ArmState armState = ArmState.Ready;
    double gripTime = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, false, null);

//        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        gripper.setPosition(1);

        telemetry.addData("Angle:", Math.abs(robot.armSubsystem.getAngle() - 7));
        telemetry.addData("Current Position:", armExtend.getCurrentPosition());

        robot.armSubsystem.setWristAngle(0);

        if(!(Math.abs(robot.armSubsystem.getAngle() - 7) < 1 /*Angle not in position*/)) {
            armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
            telemetry.addData("Moving Arm", "True");
        } else {
            armAngle.setPower(0);
            telemetry.addData("Moving Arm", "False");
        }

        if(!(armExtend.getCurrentPosition() < -220 && armExtend.getCurrentPosition() > -228 /*Length not in position*/)) {
            armExtend.setPower((armExtend.getCurrentPosition() - -224 )/ -10.0);
            telemetry.addData("Moving Angle", "True");
        } else {
            armExtend.setPower(0);
            telemetry.addData("Moving Angle", "False");
        }

        if(gamepad2.a) {
            armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void start() {
//        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        robot.holoDrivetrain.smoothDrive(x, y, gamepad1.right_stick_x);

//        if(gamepad1.a || gamepad2.a) {
//            intake.setPower(-1);
//        } else if (gamepad1.b || gamepad2.b) {
//            intake.setPower(0);
//        }

        if(gamepad2.a) {
            if(!toggleDown) {
                toggleDown = true;
                toggleIntake = !toggleIntake;
                double power = toggleIntake ? -1 : 0;
                intake.setPower(power);
            }
        } else {
            toggleDown = false;
        }

//        if(gamepad1.x) {
//            shotRelease.setPosition(0);
//        } else if (gamepad1.y) {
//            shotRelease.setPosition(0.1);
//        }

        if(gamepad1.x) {
            slidePush.setPosition(0);
        } else if (gamepad1.y) {
            slidePush.setPosition(0.1);
        }

        if(gamepad2.x) {
            gripper.setPosition(0);
        } else if (gamepad2.y) {
            gripper.setPosition(1);
        }

        if (gamepad2.dpad_up) {
            lift.setPower(1);
        } else if (gamepad2.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad2.right_bumper) {
            wrist.setPosition(0);
        } else if (gamepad2.left_bumper) {
            wrist.setPosition(1);
        }

        if(gamepad2.dpad_left) {
            liftRaise.setPosition(0);
        } else if (gamepad2.dpad_right) {
            liftRaise.setPosition(0.1);
        }

//        armAngle.setPower(-gamepad2.left_stick_y);
//        armExtend.setPower(gamepad2.right_stick_y);

        if(gamepad2.start && !toggleStart) {
            toggleStart = true;
            autoWrist = !autoWrist;

        } else if(!gamepad2.start && toggleStart) {
            toggleStart = false;
        }

        if(autoWrist) {
            robot.armSubsystem.setAbsoluteWristAngle(30);
        }

        switch (armState) {
            case Ready:
                if(gamepad2.triangle /*Button to initiate grabbing pixel*/) {
                    armState = ArmState.LowerArm;
                }
                if(gamepad2.left_stick_y < 0 /*Stick Pushed Up*/) {
                    armState = ArmState.FreeMovement;
                }
                break;
            case LowerArm:
                armAngle.setPower(-0.5);
                if(Math.abs(robot.armSubsystem.getAngle() - 2) < 1) {
                    armAngle.setPower(0);
                    armState = ArmState.Grip;
                }
                break;
            case Grip:
                if(gripTime == 0) {
                    gripTime = System.currentTimeMillis();
                }
                gripper.setPosition(0);
                if(System.currentTimeMillis() - gripTime > 500) {
                    armState = ArmState.RaiseArm;
                    gripTime = 0;
                }
                break;
            case RaiseArm:
                armAngle.setPower(0.5);
                if(robot.armSubsystem.getAngle() > 23 /*Arm is raised*/) {
                    armAngle.setPower(0);
                    armState = ArmState.FreeMovement;
                }
                break;
            case FreeMovement:
                robot.armSubsystem.setAbsoluteWristAngle(30);

                if(true /*Angle Stick Pushed up and not at max angle, or down and not at min angle*/) {
                    armAngle.setPower(-gamepad2.left_stick_y);
                } else {
                    armAngle.setPower(0);
                }

                if(true /*Length Stick Pushed up and not at max length, or down and not at min length*/) {
                    armExtend.setPower(gamepad2.right_stick_y);
                } else {
                    armExtend.setPower(0);
                }

                if(gamepad2.triangle /*Button to Move arm into ready state*/) {
                    armState = ArmState.FreeReadyTransition;
                    armAngle.setPower(0);
                    armExtend.setPower(0);
                }
                break;
            case FreeReadyTransition:
                gripper.setPosition(1);
                robot.armSubsystem.setWristAngle(0);
                if(!(Math.abs(robot.armSubsystem.getAngle() - 7) < 1 /*Angle not in position*/)) {
                    armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
                    telemetry.addData("Moving Arm", "True");
                } else {
                    armAngle.setPower(0);
                    telemetry.addData("Moving Arm", "False");
                }

                if(!(armExtend.getCurrentPosition() < -220 && armExtend.getCurrentPosition() > -228 /*Length not in position*/)) {
                    armExtend.setPower((armExtend.getCurrentPosition() - -224 )/ -10.0);
                    telemetry.addData("Moving Angle", "True");
                } else {
                    armExtend.setPower(0);
                    telemetry.addData("Moving Angle", "False");
                }

                break;
        }

        telemetry.addData("Arm Length:", armExtend.getCurrentPosition());
        telemetry.addData("Arm Angle:", robot.armSubsystem.getAngle());
        telemetry.addData("State", armState);
        telemetry.update();
    }
}
