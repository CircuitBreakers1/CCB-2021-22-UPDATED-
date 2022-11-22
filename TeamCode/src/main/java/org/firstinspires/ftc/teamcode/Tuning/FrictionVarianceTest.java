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

package org.firstinspires.ftc.teamcode.Tuning;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.ArrayList;

@Autonomous(name = "Friction Variance", group = "Tuning")
public class FrictionVarianceTest extends LinearOpMode {
    Robot robot = new Robot(this, false, false);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, 0, 0, 0, false);

        //Set the motors zero power behavior to float
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Display telemetry showing the robot is ready to start
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Set the motors to run at full power
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(1);

        for (int i = 0; i < 200; i++) {
            telemetry.addData("Left Front", leftFront.getCurrentPosition());
            telemetry.addData("Left Back", leftBack.getCurrentPosition());
            telemetry.addData("Right Front", rightFront.getCurrentPosition());
            telemetry.addData("Right Back", rightBack.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        //Stop the motors and create an int of their encoder values named launchStart
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        int[] launchStart = {leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightFront.getCurrentPosition(), rightBack.getCurrentPosition()};

        sleep(2000);

        double[] launchEnd = {leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightFront.getCurrentPosition(), rightBack.getCurrentPosition()};


        double[] launchTravel = {launchEnd[0] - launchStart[0], launchEnd[1] - launchStart[1], launchEnd[2] - launchStart[2], launchEnd[3] - launchStart[3]};

        //Get the smallest launch travel value
        double smallestLaunchTravel = Math.min(Math.min(launchTravel[0], launchTravel[1]), Math.min(launchTravel[2], launchTravel[3]));

        //Get the launch travel variance
        double[] ltVariance = {smallestLaunchTravel / launchTravel[0], smallestLaunchTravel / launchTravel[1], smallestLaunchTravel / launchTravel[2], smallestLaunchTravel / launchTravel[3]};

        //Display all data on telemetry
        telemetry.addData("Launch Travel", "Left Front: %d, Left Back: %d, Right Front: %d, Right Back: %d", launchTravel[0], launchTravel[1], launchTravel[2], launchTravel[3]);
        telemetry.addData("Launch Travel Variance", "Left Front: %f, Left Back: %f, Right Front: %f, Right Back: %f", ltVariance[0], ltVariance[1], ltVariance[2], ltVariance[3]);
        telemetry.update();
    }
}
