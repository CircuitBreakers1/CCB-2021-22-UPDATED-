package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.slidePush;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.viperTouch;
import static org.firstinspires.ftc.teamcode.Tuning.OldAutoTuning.ARMANGLE;
import static org.firstinspires.ftc.teamcode.Tuning.OldAutoTuning.ARMANGLE2;
import static org.firstinspires.ftc.teamcode.Tuning.OldAutoTuning.ARMLENGTH;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMBASE;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@Autonomous(name = "PickupTest", group = "Auto")
public class PickupTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 robot = new Robot2023();

        robot.init(hardwareMap, true, this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashTele = dashboard.getTelemetry();

        boolean armTriggered = false, touchTriggered = false;

        while(opModeInInit()) {
            if (!(abs(robot.armSubsystem.getAngle() - 10) < 4) /*Angle not in position*/ && !armTriggered) {
                armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
            } else {
                armAngle.setPower(0);
                armTriggered = true;
            }
            if(armTriggered) {
                if (!touchTriggered) {
                    if (!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */) {
                        armExtend.setPower(0);
                        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        touchTriggered = true;
                    }
                    armExtend.setPower(0.65);
                } else {
                    armExtend.setTargetPosition(ARMBASE);
                    armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtend.setPower(0.8);
                }
            }
        }

        waitForStart();

        gripper.setPosition(1);
        //Extend Arm
        armExtend.setTargetPosition(ARMLENGTH);
        while(opModeIsActive() && (armExtend.getCurrentPosition() < ARMLENGTH - 50 || armExtend.getCurrentPosition() > ARMLENGTH + 50)) {
            idle();
        }
        //Lower Arm
        double armTargetAngle = ARMANGLE;
        while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
            armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
        }
        armAngle.setPower(0);
        //Grip
        gripper.setPosition(0);
        sleep(500);
        //Raise Arm
        armTargetAngle = ARMANGLE2;
        while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
            armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
        }
        armAngle.setPower(0);
        //Retract Arm
        armExtend.setTargetPosition(ARMBASE);
        while(opModeIsActive() && (armExtend.getCurrentPosition() < ARMBASE - 50 || armExtend.getCurrentPosition() > ARMBASE + 50)) {
            idle();
        }
        gripper.setPosition(1);
        //Sweep
        slidePush.setPosition(0.77);
        sleep(1500);
        slidePush.setPosition(0.64);

        while(opModeIsActive());
    }
}