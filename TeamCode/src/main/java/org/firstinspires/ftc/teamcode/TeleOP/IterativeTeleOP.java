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

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import static org.firstinspires.ftc.teamcode.Subsystems.ButtonToggleSubsystem.updateButtons;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name = "TeleOP")
public class IterativeTeleOP extends OpMode {
    Robot robot = new Robot(this, false);
    Orientation angles;
    boolean isIntaking = false;
    boolean isOutputting = false;
    boolean manualControl = false;
    int[] armLimits = {100, 1000, 1500};

    //Move into init??
    Telemetry.Item intake = telemetry.addData("Is intaking?", isIntaking);
    Telemetry.Item output = telemetry.addData("Is outputting?", isOutputting);

    //ButtonToggleSubsystem a1 = new ButtonToggleSubsystem(() -> gamepad1.a);

    @Override
    public void init() {
        robot.init(hardwareMap, 0, 0, 0, false);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        leftBack.resetEncoder();
        leftFront.resetEncoder();
        rightFront.resetEncoder();
        rightBack.resetEncoder();
    }

    @Override
    public void loop() {
            holOdom.updatePose();
            Pose2d currentLocation = holOdom.getPose();
            double angle = currentLocation.getHeading();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            telemetry.addData("Arm Position", armLift.getCurrentPosition());
            telemetry.addData("Robot X", currentLocation.getX());
            telemetry.addData("Robot Y", currentLocation.getY());
            telemetry.addData("Robot Angle", angle);
            telemetry.addData("Gyro Angle", angles.firstAngle);

            telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            telemetry.addData("Back Odo", backOdo.getCurrentPosition());
            telemetry.addData("Arm Spot", armLift.getCurrentPosition());


            //drive.driveRobotCentric(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            //drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -angle);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFront.set(0.75 * (y + x + rx));
            leftBack.set(0.75 * (y - x + rx));
            rightFront.set(0.75 * (y - x - rx));
            rightBack.set(0.75 * (y + x - rx));

            if(!isIntaking && gamepad2.a) {
                pickupLeft.setPower(1);
                pickupRight.setPower(-1);
                isIntaking = true;
                isOutputting = false;
            }
            if(!isOutputting && gamepad2.b) {
                pickupLeft.setPower(-1);
                pickupRight.setPower(1);
                isOutputting = true;
                isIntaking = false;
            }
            if(gamepad2.x || (!coneTouch.getState() && isIntaking)) {
                pickupLeft.setPower(0);
                pickupRight.setPower(0);
                isIntaking = false;
                isOutputting = false;
            }


            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad2.left_stick_y != 0) {
                if(!manualControl) {
                    manualControl = true;
                    armLift.setPower(0);
                    armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                armLift.setPower(gamepad2.left_stick_y);

            } else if (manualControl) {
                armLift.setTargetPosition(armLift.getCurrentPosition());
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(0.75);
            }

            if(!manualControl) {
                if (gamepad2.dpad_up) {
                    armLift.setTargetPosition(armLimits[2]);
                    armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLift.setPower(0.75);
                } else if (gamepad2.dpad_left) {
                    armLift.setTargetPosition(armLimits[1]);
                    armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLift.setPower(0.75);
                } else if (gamepad2.dpad_down) {
                    armLift.setTargetPosition(armLimits[0]);
                    armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLift.setPower(0.75);
                }
            }
    }

    @Override
    public void stop() {
        //Make sure the robot controller clears any actions setup by the telemetry so they do not
        //run when the opmode has stopped
        telemetry.clearAll();
    }

}
