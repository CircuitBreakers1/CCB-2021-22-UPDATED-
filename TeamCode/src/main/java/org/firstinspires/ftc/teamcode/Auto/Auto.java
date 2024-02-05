package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.ConfigSetting.DELAY_SECONDS;
import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.ConfigSetting.START_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.base;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.shotRelease;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.viperTouch;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.wrist;
import static org.firstinspires.ftc.teamcode.Tuning.OldAutoTuning.x1;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMBASE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMPICKUPANGLE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.GRIPCLOSED;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.GRIPOPEN;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.ConfigSetting;
import org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector;
import org.firstinspires.ftc.teamcode.Subsystems.ColorDetectionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MovementSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PoseSupply;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@Autonomous(name = "OldAuto", group = "Auto")
@Disabled
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 robot = new Robot2023();

        robot.init(hardwareMap, true, this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashTele = dashboard.getTelemetry();
        //this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

//        telemetry = FtcDashboard.getInstance().getTelemetry();


        AutoSwitcher autoSwitcher = new AutoSwitcher(robot.movementSubsystem);
        ConfigSetting currentConfiguration = START_LOCATION;

        ColorBlobDetector.PropColor currentColor = ColorBlobDetector.PropColor.BLUE;
        robot.cameraSubsystem.setColorBlobDetector(currentColor);

        ColorBlobDetector.PropGuess propGuess = ColorBlobDetector.PropGuess.UNKNOWN;

        ArmSubsystem.ArmState armState = ArmSubsystem.ArmState.Ready;

        //Represents whether robot states are ready:
        //Indexes: 0 - Arm Pickup Maneuver Happened, 1 - Right Bay Shows YELLOW, 2 - Vision Target Acquired, 3 - Odo not in Timbucktwo, 4 - Arm Length Zeroed, 5 - Arm Angle Zeroed
        boolean[] systemStates = {false, false, false, false, false, false};
        boolean allSystemsGo = false;

        double gripTime = 0;

        //Up, right, down, left
        boolean[] dpadPressed = {false, false, false, false};

        holOdom.updatePose(autoSwitcher.getStartLocation().startPose);

        //Telemetry dash = FtcDashboard.getInstance().getTelemetry();

        wrist.setPosition(0);

        //TODO: Implement Lights to Show Ready Status
        //Init Loop
        while (opModeInInit()) {
            //Do Arm Zeroing
            if (!(abs(robot.armSubsystem.getAngle() - 10) < 4) /*Angle not in position*/ && !systemStates[5]) {
                armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
            } else {
                armAngle.setPower(0);
                systemStates[5] = true;
            }

            if (!systemStates[4]) {
                if (!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */) {
                    armExtend.setPower(0);
                    armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    systemStates[4] = true;
                }
                armExtend.setPower(0.65);
            } else {
                armExtend.setTargetPosition(ARMBASE);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(0.8);
            }


            //Perform Arm Maneuver
            switch (armState) {
                case Ready:
                    gripper.setPosition(GRIPCLOSED);
                    robot.armSubsystem.setWristAngle(0);
                    if (gamepad1.a && systemStates[4] && systemStates[5]) {
                        armState = ArmSubsystem.ArmState.LowerArm;
                    }
                    break;
                case LowerArm:
                    gripper.setPosition(GRIPOPEN);
                    armAngle.setPower(-0.85);
                    if (abs(robot.armSubsystem.getAngle() - ARMPICKUPANGLE) < 1) {
                        armAngle.setPower(0);
                        armState = ArmSubsystem.ArmState.Grip;
                    }
                    break;
                case Grip:
                    if (gripTime == 0) {
                        gripTime = System.currentTimeMillis();
                    }
                    gripper.setPosition(GRIPCLOSED);
                    if (System.currentTimeMillis() - gripTime > 500) {
                        armState = ArmSubsystem.ArmState.RaiseArm;
                        gripTime = 0;
                    }
                    break;
                case RaiseArm:
                    armAngle.setPower(0.85);
                    if (robot.armSubsystem.getAngle() > 10 /*Arm is raised*/) {
                        armAngle.setPower(0);
                        armState = ArmSubsystem.ArmState.Ready;
                        armAngle.setPower(0);
                        //armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armExtend.setTargetPosition(armExtend.getCurrentPosition());
                        systemStates[0] = true;
                    }
                    break;
            }


            //Configure auto setup. Dpad up/down to toggle through values, dpad left/right to cycle through options
            if (gamepad1.dpad_up) {
                if (!dpadPressed[0]) {
                    dpadPressed[0] = true;
                    autoSwitcher.toggleUp(currentConfiguration);
                    if (currentConfiguration == START_LOCATION) {
                        switch (autoSwitcher.getAlliance()) {
                            case RED:
                                currentColor = ColorBlobDetector.PropColor.RED;
                                break;
                            case BLUE:
                            default:
                                currentColor = ColorBlobDetector.PropColor.BLUE;
                                break;
                        }
                        robot.cameraSubsystem.setColorBlobDetector(currentColor);
                    }
                }
            } else {
                dpadPressed[0] = false;
            }
            if (gamepad1.dpad_down) {
                if (!dpadPressed[2]) {
                    dpadPressed[2] = true;
                    autoSwitcher.toggleDown(currentConfiguration);
                    if (currentConfiguration == START_LOCATION) {
                        switch (autoSwitcher.getAlliance()) {
                            case RED:
                                currentColor = ColorBlobDetector.PropColor.RED;
                                break;
                            case BLUE:
                            default:
                                currentColor = ColorBlobDetector.PropColor.BLUE;
                                break;
                        }
                        robot.cameraSubsystem.setColorBlobDetector(currentColor);
                    }
                }
            } else {
                dpadPressed[2] = false;
            }
            if (gamepad1.dpad_left) {
                if (!dpadPressed[3]) {
                    dpadPressed[3] = true;
                    currentConfiguration = ConfigSetting.cycleDown(currentConfiguration);
                }
            } else {
                dpadPressed[3] = false;
            }
            if (gamepad1.dpad_right) {
                if (!dpadPressed[1]) {
                    dpadPressed[1] = true;
                    currentConfiguration = ConfigSetting.cycleUp(currentConfiguration);
                }
            } else {
                dpadPressed[1] = false;
            }

            if (gamepad1.triangle) {
                shotRelease.setPosition(0);
            } else if (gamepad1.square) {
                shotRelease.setPosition(0.5);
            }

            //Check System States -- Arm Maneuver updates after arm is moved, so we don't need to check that here
            systemStates[1] = robot.colorDetectionSubsystem.getBayColors()[0] == ColorDetectionSubsystem.BayColor.YELLOW;
            propGuess = robot.cameraSubsystem.getPropGuess();
            systemStates[2] = propGuess != ColorBlobDetector.PropGuess.UNKNOWN;
            systemStates[3] = true; //TODO: Im lazy rn future Lucas fix this

            //Check if all systems are ready
            for (boolean thing : systemStates) {
                if (!thing) {
                    allSystemsGo = false;
                    break;
                }
                allSystemsGo = true;
            }

            dashTele.addData("Guess", propGuess);
            dashTele.update();

            telemetry.addLine("Status: " + (allSystemsGo ? "ALL SYSTEMS GO!" : "Not Ready"));
            telemetry.addData("Current Configuration", currentConfiguration + " - " + autoSwitcher.get(currentConfiguration));
            telemetry.addData("Start Location", autoSwitcher.get(START_LOCATION));
            telemetry.addData("Delay Seconds", autoSwitcher.get(DELAY_SECONDS));
            telemetry.addData("Place Location", autoSwitcher.get(ConfigSetting.PLACE_LOCATION));
            telemetry.addData("Park Location", autoSwitcher.get(ConfigSetting.PARK_LOCATION));
            telemetry.addData("Movement Path", autoSwitcher.get(ConfigSetting.MOVEMENT_PATH));
            telemetry.addData("Prop Guess", propGuess);
            telemetry.addLine()
                    .addData("X", "%.2f", holOdom.getPose().getX())
                    .addData("Y", "%.2f", holOdom.getPose().getY())
                    .addData("Heading", "%.3f", holOdom.getPose().getHeading());
            telemetry.addLine("Status Detail:")
                    .addData("Arm Angle Set", systemStates[5])
                    .addData("Viper Zeroed", systemStates[4])
                    .addData("Arm Pickup", systemStates[0])
                    .addData("Right Bay Has YELLOW", systemStates[1])
                    .addData("Vision Target Acquired", systemStates[2])
                    .addData("Odo in Reasonable Area", systemStates[3]);
            telemetry.update();
        }

        waitForStart();

        //The code between Backdrop side and Audience side begins different, but ends the same. This switch handles the difference
        //However either way we begin with extending the arm, and a few other cleanup things from init, so we do that.
        //propGuess = ColorBlobDetector.PropGuess.LEFT;
        robot.cameraSubsystem.setColorBlobDetector(null);
        //robot.cameraSubsystem.pauseCamera();
        armExtend.setTargetPosition(-1600);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setPower(0.8);
        double armTarget = 0;
        //Create the runnable to lower the arm as we move to the spike marks.
        Runnable lowerArm = () -> {
            //Make sure the arm is far enough out
            if (armExtend.getCurrentPosition() < -400) {
                double error = armTarget - robot.armSubsystem.getAngle();
                if (abs(error) > 3) {
                    armAngle.setPower(0.85 * signum(error));
                }
            }
        };

        ColorBlobDetector.PropGuess mirroredGuess = propGuess;
        if (autoSwitcher.getAlliance() == AutoSwitcher.Alliance.RED) {
            switch (mirroredGuess) {
                case LEFT:
                    mirroredGuess = ColorBlobDetector.PropGuess.RIGHT;
                    break;
                case RIGHT:
                    mirroredGuess = ColorBlobDetector.PropGuess.LEFT;
                    break;
            }
        }

        switch (autoSwitcher.getStartLocation()) {
            case RED_BACKDROP:
            case BLUE_BACKDROP:
                //Delay
                if(autoSwitcher.getDelayMS() != 0) {
                    sleep(autoSwitcher.getDelayMS());
                }
                //Backdrop side code
                switch (mirroredGuess) {
                    case LEFT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 22, 55, -1.57, 1);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 15, 48.5, -0.685, 1);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 16, 46, -1.57, 1);
                        break;
                }
                //Finish moving the arm, then stop the arm and drop the pixel.
                while (opModeIsActive()) {
                    if (armExtend.getCurrentPosition() < -200) {
                        double error = armTarget - robot.armSubsystem.getAngle();
                        if (abs(error) > 3) {
                            armAngle.setPower(0.85 * signum(error));
                        } else {
                            break;
                        }
                    }
                }
                armAngle.setPower(0);
                //Drop the pixel.
                gripper.setPosition(GRIPOPEN);
                sleep(300);
                if (mirroredGuess == ColorBlobDetector.PropGuess.LEFT) {
                    autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 30, 55, -1.57, 1, false);
                } else if(mirroredGuess == ColorBlobDetector.PropGuess.MIDDLE || mirroredGuess == ColorBlobDetector.PropGuess.UNKNOWN) {
                    autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 16, 50, -1.57, 1, false);
                }
                //Go to the backboard, then the code merge happens
                break;
            case RED_AUDIENCE:
            case BLUE_AUDIENCE:
                //Audience side code
                switch (mirroredGuess) {
                    case LEFT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -43, 38, -3.14, 1);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -45.5, 55, -1.57, 1);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -39, 44.5, -1.57, 1);
                        break;
                }

                //Finish moving the arm, then stop the arm and drop the pixel.
                while (opModeIsActive()) {
                    if (armExtend.getCurrentPosition() < -200) {
                        double error = armTarget - robot.armSubsystem.getAngle();
                        if (abs(error) > 3) {
                            armAngle.setPower(0.85 * signum(error));
                        } else {
                            break;
                        }
                    }
                }
                armAngle.setPower(0);
                //Drop the pixel.
                gripper.setPosition(GRIPOPEN);
                sleep(300);

                while(opModeIsActive() && robot.armSubsystem.getAngle() < 10) {
                    armAngle.setPower(0.85);
                }
                armAngle.setPower(0);

                armExtend.setTargetPosition(base);

                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -44, 60, -3.14, 1);
                if(autoSwitcher.getDelayMS() != 0) {
                    sleep(autoSwitcher.getDelayMS());
                }
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 60, -3.14, 1);
                break;
        }
        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 36, -3.14, 1, false, MovementSubsystem.GRAB_PIXEL_AUTO);

        double armTargetAngle = 25;
        while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
            armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
        }
        armAngle.setPower(0);
        armExtend.setTargetPosition(-1075);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setPower(0.9);

        AutoSwitcher.PlaceLocation placeMirrored = autoSwitcher.getPlaceLocation();
        if (autoSwitcher.getAlliance() == AutoSwitcher.Alliance.RED) {
            switch (placeMirrored) {
                case LEFT:
                    placeMirrored = AutoSwitcher.PlaceLocation.RIGHT;
                    break;
                case RIGHT:
                    placeMirrored = AutoSwitcher.PlaceLocation.LEFT;
                    break;
            }
        }

        final double LEFT_TO_RIGHT;
        switch (placeMirrored) {
            case LEFT:
                LEFT_TO_RIGHT = autoSwitcher.getAlliance() == AutoSwitcher.Alliance.RED ? 0 : 0;
                break;
            case RIGHT:
                LEFT_TO_RIGHT = autoSwitcher.getAlliance() == AutoSwitcher.Alliance.RED ? 2 : 2;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + autoSwitcher.getPlaceLocation());
        }

        //Drop Pixel on Backdrop. Default points to LEFT edge of the drop zone.
        switch (mirroredGuess) {
            case LEFT:
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x1, 44 - LEFT_TO_RIGHT, -3.14, 0.5);
                break;
            case RIGHT:
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x1, 33 - LEFT_TO_RIGHT, -3.14, 0.5);
                break;
            case MIDDLE:
            default:
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x1, 39 - LEFT_TO_RIGHT, -3.14, 0.5);
        }

//        switch (autoSwitcher.getAlliance()) {
//            case RED:
//                switch (propGuess) {
//                    case LEFT:
//                        break;
//                    case RIGHT:
//                        break;
//                    case UNKNOWN:
//                    case MIDDLE:
//                        break;
//                    default:
//                        throw new IllegalStateException("Unexpected value: " + propGuess);
//                }
//                break;
//            case BLUE:
//                switch (propGuess) {
//                    case LEFT:
//                        robot.movementSubsystem.moveTo(PoseSupply.ODOMETRY, 50,43,-3.14,0.5);
//                        break;
//                    case RIGHT:
//                        robot.movementSubsystem.moveTo(PoseSupply.ODOMETRY, 50,29.5,-3.14,0.5);
//                        break;
//                    case UNKNOWN:
//                    case MIDDLE:
//                        robot.movementSubsystem.moveTo(PoseSupply.ODOMETRY, 50,39,-3.14,0.5);
//                        break;
//                    default:
//                        throw new IllegalStateException("Unexpected value: " + propGuess);
//                }
//                break;
//            default:
//                throw new IllegalStateException("Unexpected value: " + autoSwitcher.getStartLocation());
//        }

        gripper.setPosition(GRIPOPEN);
        sleep(500);
        armExtend.setTargetPosition(base);
        sleep(500);
        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 36, -3.14, 1, false);


//        int cycleAmount = 1;
//        switch (autoSwitcher.getMovementPath()) {
//            case INSIDE_TRUSS:
//            case OUTSIDE_TRUSS:
////                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 0, 50, 0, 1);
////                break;
//            case STAGE_DOOR:
//                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 11, 0, 1, false);
//                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -24, 11, PI / 8, 1, true);
////                armExtend.setTargetPosition(-1200);
////                armTargetAngle = -3;
////                while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
////                    armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
////                }
////                armAngle.setPower(0);
////                gripper.setPosition(0);
//                //robot.movementSubsystem.requestApriltagSync();
//                //Check for apriltag
//                long curTime = System.currentTimeMillis();
//                while (opModeIsActive() && System.currentTimeMillis() < curTime + 5000) {
//                    Pose2d aprilPose = robot.cameraSubsystem.getPoseFromAprilTag();
//                    if(aprilPose != null) {
//                        holOdom.updatePose();
//                        holOdom.updatePose(aprilPose);
//                        holOdom.updatePose();
//                        holOdom.updatePose(aprilPose);
//                        leftOdo.resetEncoder();
//                        rightOdo.resetEncoder();
//                        frontOdo.resetEncoder();
//                        holOdom.updatePose();
//                        telemetry.addLine("AprilTag Locked!");
//                        Pose2d realPose = holOdom.getPose();
//                        telemetry.addData("X", realPose.getX()).addData("Y", realPose.getY()).addData("Theta", realPose.getHeading());
//                        telemetry.update();
//                        //sleep(2000);
//                        break;
//                    }
//                }
//                armExtend.setTargetPosition(ARMLENGTH + 200);
//                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, -55.5, 8.75, 0, 1);
//                gripper.setPosition(1);
//                //Extend Arm
//                armExtend.setTargetPosition(ARMLENGTH);
//                while(opModeIsActive() && (armExtend.getCurrentPosition() < ARMLENGTH - 50 || armExtend.getCurrentPosition() > ARMLENGTH + 50)) {
//                    idle();
//                }
//                //Lower Arm
//                armTargetAngle = ARMANGLE;
//                while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
//                    armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
//                }
//                armAngle.setPower(0);
//                //Grip
//                gripper.setPosition(0);
//                sleep(500);
//                //Raise Arm
//                armTargetAngle = ARMANGLE2;
//                while ((abs(robot.armSubsystem.getAngle() - armTargetAngle) > 3) && opModeIsActive()) {
//                    armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - armTargetAngle));
//                }
//                armAngle.setPower(0);
//                //Retract Arm
//                armExtend.setTargetPosition(ARMBASE);
//                while(opModeIsActive() && (armExtend.getCurrentPosition() < ARMBASE - 50 || armExtend.getCurrentPosition() > ARMBASE + 50)) {
//                    idle();
//                }
////                gripper.setPosition(1);
////                //Sweep
////                slidePush.setPosition(0.77);
////                sleep(1500);
////                slidePush.setPosition(0.64);
//                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 11, 3.14, 1, false);
//                while (opModeIsActive()) {
//                    telemetry.addData("Can See Tag?", robot.cameraSubsystem.getPoseFromAprilTag() != null);
//                    telemetry.update();
//                    idle();
//                }
//                return;
//            //break;
//        }

        //Park
        switch (autoSwitcher.getParkLocation()) {
            case INSIDE:
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 15, -3.14, 1);
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 46, 15, -3.14, 1);
                break;
            case OUTSIDE:
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 36, 60.5, -3.14, 1);
                autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 46, 60.5, -3.14, 1);
                break;
        }
    }
}