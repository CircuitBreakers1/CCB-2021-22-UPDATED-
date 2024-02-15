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

import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.distance;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.shoot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;

/**
 * Demonstrates new features
 */
@TeleOp(name = "TeleOP")
public class LastLastJohnTeleOp extends OpMode {
    NewRobot2023 robot = new NewRobot2023();
    boolean toggleDown = false;
    boolean squareToggle = false;
    private Gamepad.RumbleEffect whiteRumble;
    @Override
    public void init() {
        robot.init(hardwareMap, false, null);
        robot.pixelSubsystem.initGamepadsForTeleOP(gamepad1, gamepad2);
        robot.pixelSubsystem.initArm();

        whiteRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.75, 0, 750)
     -           .addStep(0, 0.75, 750)
                .build();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.pixelSubsystem.setTeleOp();
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        robot.holoDrivetrain.smoothDrive(x, y, gamepad1.right_stick_x);

        if (gamepad1.cross) {
            if (!toggleDown) {
                toggleDown = true;
                robot.pixelSubsystem.toggleIntake();
            }
        } else if (gamepad1.circle) {
            robot.pixelSubsystem.output();
        }else {
            toggleDown = false;
        }

        if(gamepad2.square) {
            if(!squareToggle) {
                robot.pixelSubsystem.fingerOverrideBase(true);
            }
            squareToggle = true;
        } else {
            squareToggle = false;
        }

        if(gamepad1.dpad_up) {
            robot.shoot();
        } else if(gamepad1.dpad_down) {
            robot.resetShoot();
        }

        robot.pixelSubsystem.liftControl(-gamepad2.right_stick_y);
        robot.pixelSubsystem.returnArm(gamepad2.triangle);
        robot.pixelSubsystem.flipVertical(gamepad2.dpad_up || gamepad2.dpad_down);
        robot.pixelSubsystem.flipHorizontal(gamepad2.dpad_left || gamepad2.dpad_right);
        robot.pixelSubsystem.dropLeft(gamepad2.left_bumper);
        robot.pixelSubsystem.dropRight(gamepad2.right_bumper);
        robot.pixelSubsystem.setHangMode(gamepad2.circle);


        robot.pixelSubsystem.runPixelSystem();

        if(robot.pixelSubsystem.rumbleProcess()) {
            gamepad1.runRumbleEffect(whiteRumble);
            gamepad2.runRumbleEffect(whiteRumble);
        }

        holOdom.updatePose();

        Pose2d pose = holOdom.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Pixels", robot.pixelSubsystem.pixelCount);
    }
}
