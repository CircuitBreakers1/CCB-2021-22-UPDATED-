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

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem.mecDrive;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.cameraInit.NO_CAM;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.drivetrain;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rotation;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.xLoc;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.yLoc;

import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;


/**
 *
 */

@Autonomous(name="Pure Pursuit Test Auto", group="Linear Opmode")
@Disabled
public class PurePursuitTest extends LinearOpMode {


    Robot robot = new Robot(this, true, false);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, 36, 7, 90, NO_CAM);

        telemetry.addData("X Loc", xLoc);
        telemetry.addData("Y Loc", yLoc);
        telemetry.addData("Heading", rotation);
        telemetry.update();

        Waypoint startLocation = new StartWaypoint(holOdom.getPose());
        Waypoint coneSquareCenterEnd =
                new EndWaypoint(36, 55, 3.14/4, 0.5, 0.5,6, 2, 3.14/12);


        Waypoint conePickUp =
                new EndWaypoint(3, 55, 3.14, 0.5, 0.5, 6, 2, 3.14/12);
        Waypoint coneSquareCenter =
                new GeneralWaypoint(36, 55, 3.14/4, 0.5, 0.5, 6);
        Waypoint coneSet =
                new EndWaypoint(42, 61, 3.14/4, 0.5, 0.5, 6, 2, 3.14/12);
        Waypoint currentRobotLocation;

        Path initialPath = new Path(startLocation, coneSquareCenterEnd);
        initialPath.init();

        waitForStart();

        initialPath.followPath(mecDrive, holOdom);

        


        drivetrain.stop();
    }
}
