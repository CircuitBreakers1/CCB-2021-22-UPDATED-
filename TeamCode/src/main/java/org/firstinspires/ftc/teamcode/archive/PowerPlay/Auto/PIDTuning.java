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

package org.firstinspires.ftc.teamcode.archive.PowerPlay.Auto;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.getRemainingTime;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.moveTo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.NO_CAM;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.holOdom;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot;


/**
 *  The autonomous for the Left side of the field.
 */

@Autonomous(name="PID Tuning", group="Linear Opmode")
@Disabled
public class PIDTuning extends LinearOpMode {


    Robot robot = new Robot(this, true, true);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, 35.8, 7, 90, NO_CAM);

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();


        while (opModeInInit()) {
            telemetry.addData("X Loc", moving.getX());
            telemetry.addData("Y Loc", moving.getY());
            telemetry.addData("Heading", moving.getHeading());
            telemetry.update();
        }

        moveTo(36,47, 0.7);


    }
}
