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

import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.frontStage;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.intake;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftRotate;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.rightRotate;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.LEFTPOS;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.RIGHTPOS;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.intakePower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;
import org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023;

/**
 * Demonstrates new features
 */
@TeleOp(name = "TeleOP", group = "")
public class LastLastJohnTeleOp extends OpMode {

    NewRobot2023 robot = new NewRobot2023();

    boolean toggleDown = false;
    boolean toggleIntake = false;


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

//        rightRotate.setPosition(RIGHTPOS);
//        leftRotate.setPosition(LEFTPOS);

        //Ground 0.76
        //Up 0.7
        //5 Stack 0.735
        //4 Stack 0.74
        //3 Stack 0.75
        //2 Stack 0.755
        frontStage.setPosition(tuningConstants2023.frontStage);

        //Right Base 0.195
        //Left Base 0.49

//        if(gamepad2.a) {
//            rightRotate.setPosition(0.195);
//            leftRotate.setPosition(0.49);
//        } else if (gamepad2.b) {
//            rightRotate.setPosition(0.195 + ARMROTATECONST);
//            leftRotate.setPosition(0.49 + ARMROTATECONST);
//        }
//        rightRotate.setPosition(ARMROTATECONST);

        if (gamepad2.a) {
            if (!toggleDown) {
                toggleDown = true;
                toggleIntake = !toggleIntake;
                double power = toggleIntake ? intakePower : 0;
                intake.setPower(power);
            }
        } else if (gamepad2.left_bumper) {
            toggleIntake = true;
            intake.setPower(-intakePower);
        }else {
            toggleDown = false;
        }

        robot.holoDrivetrain.smoothDrive(x, y, gamepad1.right_stick_x);

        robot.liftSubsystem.simpleLift(-gamepad2.left_stick_y);
    }



}
