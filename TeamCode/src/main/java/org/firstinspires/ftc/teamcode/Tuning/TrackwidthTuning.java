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

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.backOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.drivetrain;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.imu;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightOdo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.ArrayList;

@Autonomous(name = "Track Speed Tuning", group = "Tuning")
public class TrackwidthTuning extends LinearOpMode {
    Robot robot = new Robot(this, false, false);

    double acceptableErrorPercent = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, 0, 0, 0, false);

        waitForStart();

        //Add status for the test
        telemetry.addData("Status", "Running");
        telemetry.update();

        //Reset encoders
        drivetrain.resetEncoders();
        leftOdo.resetEncoder();
        rightOdo.resetEncoder();
        backOdo.resetEncoder();

        //Set the motors to run at 0.5 power
        leftFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);

        double startTime = getRuntime();
        double endTime = startTime + 2;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        double startAngle = angles.firstAngle;
        double deltaAngle = angle - startAngle;

        while (getRuntime() < endTime) {
            angles = imu.getAngularOrientation();
            angle = angles.firstAngle;
            deltaAngle =  angle - startAngle;
            telemetry.addData("Angle", angle);
            telemetry.addData("Delta Angle", deltaAngle);
            telemetry.update();

            //If the angle is greater than 90 degrees, slow down the motors to 0.2
            if (angle > 90) {
                leftFront.setPower(-0.2);
                leftBack.setPower(-0.2);
                rightFront.setPower(0.2);
                rightBack.setPower(0.2);
            }
        }

        drivetrain.stop();

        double ticksToInches = Robot.ticksToIn;
        double trackwidth = ((leftOdo.getCurrentPosition() * ticksToInches) + (rightOdo.getCurrentPosition() * ticksToInches)) / deltaAngle;
        double offset = (4 * ticksToInches * backOdo.getCurrentPosition()) / deltaAngle;

        telemetry.addData("Trackwidth", trackwidth);
        telemetry.addData("Offset", offset);
        telemetry.update();


        while (opModeIsActive()) {
            telemetry.addData("Trackwidth", trackwidth);
            telemetry.addData("Offset", offset);
            telemetry.update();
        }
    }
}
