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
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMPICKUPANGLE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.GRIPCLOSED;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.GRIPOPEN;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.shootAngle;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmState;
import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

/**
 * Demonstrates new features
 */
@TeleOp(name = "TeleOP", group = "")
public class LastLastJohnTeleOp extends OpMode {

    NewRobot2023 robot = new NewRobot2023();



    @Override
    public void init() {
        robot.init(hardwareMap, false, null);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        robot.holoDrivetrain.smoothDrive(x, y, gamepad1.right_stick_x);

        robot.liftSubsystem.simpleLift(-gamepad2.left_stick_y);
    }



}
