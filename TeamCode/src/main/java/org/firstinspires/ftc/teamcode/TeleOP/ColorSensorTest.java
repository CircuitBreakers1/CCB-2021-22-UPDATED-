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

import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.frontOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftFinger;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftLift;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.rightOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.rotate;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.shoot;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.touch;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.Tuning.NewTuning.colorThresh;
import static org.firstinspires.ftc.teamcode.Tuning.NewTuning.shooter;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.rotater;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Color Sensor Test", group = "Concept")
public class ColorSensorTest extends OpMode {
    NewRobot2023 robot = new NewRobot2023();
    FtcDashboard dashboard;
    Telemetry telemetry1;
    int pixelCount = 0;
    int timestammp = 0;
    boolean detected;

    @Override
    public void init() {
        robot.init(hardwareMap,false, null);
        dashboard = FtcDashboard.getInstance();
        telemetry1 = dashboard.getTelemetry();
    }

    @Override
    public void init_loop() {
//        through.setPosition(0.475);
//        through.setPosition(0.525);
    }

    @Override
    public void start() {
        robot.pixelSubsystem.toggleIntake();
        robot.pixelSubsystem.runPixelSystem();
    }

    @Override
    public void loop() {
        double colorDist = ((DistanceSensor) NewRobot2023.colorSensor).getDistance(DistanceUnit.CM);
        if(colorDist < colorThresh) {
            if(!detected) {
                pixelCount++;
            }
            detected = true;
        } else {
            detected = false;
        }
        telemetry1.addData("Threshold", colorThresh);
        telemetry1.addData("Color Distance", colorDist);
        telemetry1.addData("Detected?", detected);
        telemetry1.addData("Pixel Count", pixelCount);
        telemetry1.update();
    }
}
