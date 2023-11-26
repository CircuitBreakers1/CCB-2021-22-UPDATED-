package org.firstinspires.ftc.teamcode.archive.PowerPlay.Auto;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.ConeStack;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.High;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Medium;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem.LiftTarget.Min;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.getRemainingTime;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.moveTo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.apriltagSleevePipeline;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.armLift;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.armTouch;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.APRIL_SLEEVE;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.coneTouch;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.holOdom;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.pickupLeft;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.pickupRight;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.ColorSleevePipeline;

@Disabled
public class MainAuto extends AutoSwitcher {
    autoStartSpot startSpot;
    LinearOpMode opMode;
    Robot robot;
    Pose2d moving;
    ColorSleevePipeline.SignalColor color;

    int park = 0;


    public MainAuto(autoStartSpot givenStart, LinearOpMode givenOpMode) {
        startSpot = givenStart;
        opMode = givenOpMode;
        robot = new Robot(opMode, true, true);
    }

    public void initAuto() {
        this.robot.init(opMode.hardwareMap, 35.8, 7, 90, APRIL_SLEEVE);
        setMirror(35.8, 7, 90);
        setStartSpot(startSpot);

        holOdom.updatePose();
        moving = holOdom.getPose();

        this.opMode.telemetry.addData("X Loc", moving.getX());
        this.opMode.telemetry.addData("Y Loc", moving.getY());
        this.opMode.telemetry.addData("Heading", moving.getHeading());
        this.opMode.telemetry.addData("Location Guess", apriltagSleevePipeline.getPrediction());
        try {
            this.opMode.telemetry.addData("Tag ID", apriltagSleevePipeline.getLatestDetections().get(0).id);
        } catch (Exception e) {
            this.opMode.telemetry.addData("Tag ID", "No tag detected");
        }
        try {
            park = apriltagSleevePipeline.getPrediction().getValue();
        } catch (Exception ignored) {

        }
        this.opMode.telemetry.update();


        while (this.opMode.opModeInInit()) {
            if(!armTouch.getState() && armLift.getCurrentPosition() != 0) {
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            this.opMode.telemetry.addData("X Loc", moving.getX());
            this.opMode.telemetry.addData("Y Loc", moving.getY());
            this.opMode.telemetry.addData("Heading", moving.getHeading());
            this.opMode.telemetry.addData("Arm Value", armLift.getCurrentPosition());
            this.opMode.telemetry.addData("Location Guess", apriltagSleevePipeline.getPrediction());
            try {
                this.opMode.telemetry.addData("Tag ID", apriltagSleevePipeline.getLatestDetections().get(0).id);
            } catch (Exception e) {
                this.opMode.telemetry.addData("Tag ID", "No tag detected");
            }
            try {
                park = apriltagSleevePipeline.getPrediction().getValue();
            } catch (Exception ignored) {

            }

            this.opMode.telemetry.update();
        }
    }

    public void runAuto() {
        double startTime = this.opMode.getRuntime();

        pickupLeft.setPower(1);
        pickupRight.setPower(-1);

        while(coneTouch.getState() && this.opMode.getRuntime() - startTime < 2) {
        }

        pickupLeft.setPower(0.1);
        pickupRight.setPower(-0.1);

        LiftSubsystem.setTarget(Medium);

        //Move to the medium junction and turn
        moveToMirrored(36,47, 0.8, false);

        turnMirrored(0,0.8);

        //Move into the junction, drop and move out
        //Dropping
        moveToMirrored(39, 47, 0.6);
        pickupLeft.setPower(-1);
        pickupRight.setPower(1);
        this.opMode.sleep(1500);
        pickupLeft.setPower(0);
        pickupRight.setPower(0);
        moveToMirrored(36, 47, 0.7, false);

        //Move to the center of the cycle path and turn
        moveToMirrored(36, 59.5, 0.7, false);
        turnMirrored(180, 0.5);

        LiftSubsystem.setTarget(ConeStack);

        //Cycle timing information
        //TODO: Tune this
        int conesInStack = 5;
        int[] coneLevel = {250, 180, 140};
        double pickupTime = 2.5; //Time to lineup and pickup cone when starting in the lined square
        double movementTime = 4.5; //Time to move from the lined square to location of the junction
        double dropTime = 0.5; //Time to line up and drop the cone
        double[] parkFromLine = {0.5, 2, 2.25}; //Time to park from lined square
        double[] parkFromJunction = {2.25, 1.75, 1}; //Time to park from junction

        //Begin the cycle loop
        while(this.opMode.opModeIsActive()) {
            //Set arm height and go to cone stack
            LiftSubsystem.setTarget(ConeStack);
            //moveTo(16, 60, 0.5);

            //Check if there is time to pickup cone, otherwise park
            if(getRemainingTime(startTime, this.opMode.getRuntime()) < (pickupTime + parkFromLine[(int) mirror(park, 1)])) {
                break;
            }

            //Pickup the cone
            moveToMirrored(24, 60, 0.8, false);
            if(conesInStack == 5) {
                moveToMirrored(10.5, 59.25, 0.4);
            } else {
                moveToMirrored(9.25, 60.25, 0.4);
            }
            pickupLeft.setPower(1);
            pickupRight.setPower(-1);
            LiftSubsystem.setPosition(coneLevel[5 - conesInStack]);

            while(coneTouch.getState() && !LiftSubsystem.isAtTarget()) {
                LiftSubsystem.updatePositional();
            }

            conesInStack--;

            pickupLeft.setPower(0.1);
            pickupRight.setPower(-0.1);

            LiftSubsystem.setTarget(High);

            while(this.opMode.opModeIsActive() && armLift.getCurrentPosition() < 700) {

            }
//
//            while(armLift.getCurrentPosition() < Intake.getPosition()) {
//                LiftSubsystem.updatePositional();
//            }
//
//            moveTo(16, 60, 0.5);

//            LiftSubsystem.setTarget(Min);

            //Check if there is time to drop off cone, otherwise park
            if(getRemainingTime(startTime, this.opMode.getRuntime()) < dropTime + movementTime + parkFromJunction[(int) mirror(park, 1)]) {
                break;
            }

            //Drop off the cone
            moveToMirrored(47.5, 58.25, 0.7);
            turnMirrored(90, 0.7);
            moveToMirrored(47.5, 60.25, 0.7);
            pickupLeft.setPower(-1);
            pickupRight.setPower(1);
            double startDrop = this.opMode.getRuntime();
            while (this.opMode.getRuntime() - startDrop < 1.5) {
                LiftSubsystem.updatePositional();
            }
            pickupLeft.setPower(0);
            pickupRight.setPower(0);
            moveToMirrored(48, 60, 0.7, false);

            //Check if there is still time to cycle, otherwise park
            if(getRemainingTime(startTime, this.opMode.getRuntime()) < movementTime + pickupTime + parkFromLine[(int) mirror(park, 1)]) {
                break;
            }

            //Make sure there is still a cone in the stack
            if(conesInStack <= 0) {
                break;
            }

            turnMirrored(180, 0.7);
        }

        this.opMode.sleep(500);

        LiftSubsystem.setTarget(Min);

        //Park in designated spot
        holOdom.updatePose();
        moving = holOdom.getPose();
        double x = moving.getX(), y = moving.getY();

        LiftSubsystem.setTarget(Min);
        switch (park) {
            case 2:
                moveToMirrored(12.5, 58, 0.7);
                break;
            case 1:
                moveToMirrored(35, 58, 0.7);
                break;
            case 0:
                moveToMirrored(60, 56, 0.7);
                break;
        }

        //turn(90, 0.75);

        //Allow the arm to stabilize
        while (this.opMode.opModeIsActive()) {
            LiftSubsystem.updatePositional();
            if (armLift.getCurrentPosition() < 10) {
                break;
            }
        }
    }
}

enum autoStartSpot {
    LEFT, RIGHT
}