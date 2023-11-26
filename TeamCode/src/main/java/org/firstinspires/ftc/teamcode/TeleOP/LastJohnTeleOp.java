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
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.base;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.imu;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.intake;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.lift;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.liftRaise;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.shooterRaise;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.shotRelease;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.slidePush;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.viperTouch;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.wrist;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMBASE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMD;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMP;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
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

    //Internal Variables
    boolean toggleStart = false, autoWrist = false;
    boolean toggleIntake = false, toggleDown = false;
    boolean toggleSweep = false, toggleSweepDown = false;
    boolean breakMode = false;

    boolean extendZeroed = false;



    PIDController armPID = new PIDController(ARMP, ARMI, ARMD);
    ArmState armState = ArmState.Ready;
    double gripTime = 0;

    //Live Configuration Variables
    boolean driveField = false;
    boolean driveDown = false;
    boolean debugTelemetry = false, debugDown = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, false, null);

        wrist.setPosition(1);

        armPID.setSetPoint(-224);

        shooterRaise.setPosition(0);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Angle Error:", abs(robot.armSubsystem.getAngle() - 7));
        telemetry.addData("Current Position:", armExtend.getCurrentPosition());
        telemetry.addData("Moving Arm", extendZeroed ? "Moving to " + base : "Zeroing");
        telemetry.addData("Touch State", viperTouch.getState());

        robot.armSubsystem.setWristAngle(0);

        if(!(abs(robot.armSubsystem.getAngle() - 7) < 1 /*Angle not in position*/)) {
            armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
            telemetry.addData("Moving Arm", "True");
        } else {
            armAngle.setPower(0);
            telemetry.addData("Moving Arm", "False");
        }

        if(!extendZeroed) {
            if(!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */ ) {
                armExtend.setPower(0);
                armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendZeroed = true;
            }
            armExtend.setPower(0.25);
        } else {
            armExtend.setTargetPosition(base);
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtend.setPower(0.8);
        }
    }

    @Override
    public void start() {
        armExtend.setPower(0);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.start) {
            if(!driveDown) {
                driveField = !driveField;
                driveDown = true;
            }
        } else {
            driveDown = false;
        }

        if(gamepad1.back) {
            if(!debugDown) {
                debugTelemetry = !debugTelemetry;
                debugDown = true;
            }
        } else {
            debugDown = false;
        }


        telemetry.addData("Drive Mode", driveField ? "Field Centric" : "Robot Centric");

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = driveField ? x * Math.cos(-botHeading) - y * Math.sin(-botHeading) : x;
        double rotY = driveField ? x * Math.sin(-botHeading) + y * Math.cos(-botHeading) : y;
        robot.holoDrivetrain.smoothDrive(rotX, rotY, -gamepad1.right_stick_x);

        if(gamepad1.dpad_up) {
            shooterRaise.setPosition(0.4);
        } else if (gamepad1.dpad_down) {
            shooterRaise.setPosition(0);
        }

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

        if(gamepad1.dpad_left) {
            shotRelease.setPosition(0);
        } else if (gamepad1.dpad_right) {
            shotRelease.setPosition(0.5);
        }

        if(gamepad2.b) {
            if(!toggleSweepDown) {
                toggleSweepDown = true;
                toggleSweep = !toggleSweep;
            }
        } else {
            toggleSweepDown = false;
        }

        if(!toggleSweep) {
            slidePush.setPosition(0.64);
        } else {
            slidePush.setPosition(0.77);
        }

        if (gamepad2.dpad_up) {
            lift.setPower(1);
        } else if (gamepad2.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad2.dpad_left) {
            liftRaise.setPosition(0);
        } else if (gamepad2.dpad_right) {
            liftRaise.setPosition(0.1);
        }

        switch (armState) {
            case Ready:
                gripper.setPosition(1);
                gamepad2.setLedColor(0,0,255,-1);
                if(gamepad2.triangle /*Button to initiate grabbing pixel*/) {
                    armState = ArmState.LowerArm;
                }
                if(gamepad2.left_stick_y < 0 /*Stick Pushed Up*/) {
                    armState = ArmState.FreeMovement;
                }
                break;
            case LowerArm:
                gamepad2.setLedColor(255,0,0,-1);
                armAngle.setPower(-0.85);
                if(abs(robot.armSubsystem.getAngle() - 2) < 1) {
                    armAngle.setPower(0);
                    armState = ArmState.Grip;
                }
                break;
            case Grip:
                gamepad2.setLedColor(255,0,0,-1);
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
                gamepad2.setLedColor(255,0,0,-1);
                armAngle.setPower(0.85);
                if(robot.armSubsystem.getAngle() > 23 /*Arm is raised*/) {
                    armAngle.setPower(0);
                    armState = ArmState.FreeMovement;
                    armAngle.setPower(0);
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armExtend.setTargetPosition(armExtend.getCurrentPosition());
                    breakMode = false;
                }
                break;
            case FreeMovement:
                gamepad2.setLedColor(0,255,0,-1);
                robot.armSubsystem.setAbsoluteWristAngle(30);
                //armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(!viperTouch.getState() && armExtend.getCurrentPosition() != 0) {
                    armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(gamepad2.square) {
                    gripper.setPosition(1);
                }

                if(true /*Angle Stick Pushed up and not at max angle, or down and not at min angle*/) {
                    armAngle.setPower(-gamepad2.left_stick_y);
                } else {
                    armAngle.setPower(0);
                }

                double armExtendPower = gamepad2.right_stick_y;

                if(armExtendPower != 0) {
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armExtend.setPower(gamepad2.right_stick_y);
                    breakMode = false;
                } else {
                    if(!breakMode) {
                        armExtend.setPower(0);
                        armExtend.setTargetPosition(armExtend.getCurrentPosition());
                        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtend.setPower(0.9);
                        breakMode = true;
                    }
                }
//
//                if(true /*Length Stick Pushed up and not at max length, or down and not at min length*/) {
//                    armExtend.setPower(gamepad2.right_stick_y);
//                } else {
//                    armExtend.setPower(0);
//                }

                if(gamepad2.triangle /*Button to Move arm into ready state*/) {
                    armState = ArmState.FreeReadyTransition;
                    armAngle.setPower(0);
                    armExtend.setPower(0);
                    armExtend.setTargetPosition(base);
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extendZeroed = false;
                }
                break;
            case FreeReadyTransition:
                gamepad2.setLedColor(255,255,0,-1);
                gripper.setPosition(1);
                robot.armSubsystem.setWristAngle(0);

                if(abs(robot.armSubsystem.getAngle() - 10) < 1 && abs(armExtend.getCurrentPosition() - base) < 10 && extendZeroed) {
                    armState = ArmState.Ready;
                }

                if(!(abs(robot.armSubsystem.getAngle() - 10) < 1 /*Angle not in position*/)) {
                    armAngle.setPower(-0.85 * Math.signum(robot.armSubsystem.getAngle() - 10));
                    telemetry.addData("Moving Arm", "True");
                } else {
                    armAngle.setPower(0);
                    telemetry.addData("Moving Arm", "False");
                }

//                if(!(armExtend.getCurrentPosition() < -220 && armExtend.getCurrentPosition() > -228 /*Length not in position*/)) {
//                    armExtend.setPower((armExtend.getCurrentPosition() - -224 )/ -10.0);
//                    telemetry.addData("Moving Angle", "True");
//                } else {
//                    armExtend.setPower(0);
//                    telemetry.addData("Moving Angle", "False");
//                }
                if(!extendZeroed) {
                    if(!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */ ) {
                        armExtend.setPower(0);
                        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendZeroed = true;
                    }
                    armExtend.setPower(0.25);
                } else {
                    armExtend.setTargetPosition(base);
                    armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtend.setPower(0.8);
                }

                break;
        }

        holOdom.updatePose();
        Pose2d pose = holOdom.getPose();

        telemetry.addData("Bay State", formatAsString(robot.colorDetectionSubsystem.getBayColors()));
        telemetry.addData("State", armState);
        if(debugTelemetry) {
            telemetry.addData("Left Bay HSV", formatAsString(robot.colorDetectionSubsystem.getLeftHSV()));
            telemetry.addData("Right Bay HSV", formatAsString(robot.colorDetectionSubsystem.getRightHSV()));
            telemetry.addData("Arm Length:", armExtend.getCurrentPosition());
            telemetry.addData("Arm Angle:", robot.armSubsystem.getAngle());
            telemetry.addData("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getHeading());
        }
        telemetry.update();
    }


    private String formatAsString(Object[] array) {
        StringBuilder result = new StringBuilder();
        for (Object o : array) {
            result.append(", ").append(o);
        }
        return result.toString();
    }
    private String formatAsString(float[] array) {
        StringBuilder result = new StringBuilder();
        for (float v : array) {
            result.append(", ").append(v);
        }
        return result.toString();
    }
}
