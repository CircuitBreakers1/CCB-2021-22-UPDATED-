package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.DELAY_SECONDS;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.MOVEMENT_PATH;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.PARK_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.PLACE_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.START_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.PlaceLocation;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.PlaceLocation.LEFT;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.distance;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.frontOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftLift;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.rightOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.shoot;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.OPEN;
import static org.firstinspires.ftc.teamcode.Tuning.NewTuning.x;
import static org.firstinspires.ftc.teamcode.Tuning.NewTuning.y;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector;
import org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher;
import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;
import org.firstinspires.ftc.teamcode.Subsystems.PoseSupply;

@Autonomous(name = "Auto")
public class NewAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewRobot2023 robot = new NewRobot2023();
        robot.init(hardwareMap, true, this);

        NewAutoSwitcher autoSwitcher = new NewAutoSwitcher(robot.movementSubsystem);

        ConfigSetting currentConfiguration = START_LOCATION;

        ColorBlobDetector.PropColor currentColor = ColorBlobDetector.PropColor.BLUE;
        robot.cameraSubsystem.setColorBlobDetector(currentColor);

        ColorBlobDetector.PropGuess propGuess = ColorBlobDetector.PropGuess.UNKNOWN;


        //Represents whether robot states are ready:
        //Indexes: 0 - Arm Pickup Maneuver Happened, 1 - Vision Target Acquired, 2 - Arm Up
        boolean[] systemStates = {false, false, false};
        boolean allSystemsGo = false;

        //Up, right, down, left
        boolean[] dpadPressed = {false, false, false, false};
        boolean armUp = false;
        boolean aPressed = false;

        holOdom.updatePose(autoSwitcher.getStartLocation().startPose);

        PIDFCoefficients coefficients = leftFront.motorEx.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.pixelSubsystem.initArm();

        while(opModeInInit()) {
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

            if(gamepad1.a) {
                if(!aPressed) {
                    aPressed = true;

                    if(!armUp) {
                        robot.pixelSubsystem.enableFullManual();
                        robot.pixelSubsystem.setFingers(OPEN, OPEN);
                        robot.pixelSubsystem.liftSet(500);
                    } else {
                        robot.pixelSubsystem.setFingers(CLOSED, CLOSED);
                        robot.pixelSubsystem.setFingerOverride(true);
                        robot.pixelSubsystem.disableFullManual();
                        systemStates[0] = true;
                    }

                    armUp = !armUp;
                }
            } else {
                aPressed = false;
            }

            if(gamepad1.square) {
                robot.shoot();
            } else if (gamepad1.triangle) {
                robot.resetShoot();
            }

            propGuess = robot.cameraSubsystem.getPropGuess();
            systemStates[1] = propGuess != ColorBlobDetector.PropGuess.UNKNOWN;

            systemStates[2] = !armUp;
            allSystemsGo = systemStates[0] && systemStates[1] && systemStates[2];

            robot.pixelSubsystem.runPixelSystem();

            telemetry.addLine("Status: " + (allSystemsGo ? "ALL SYSTEMS GO!" : "Not Ready"));
            telemetry.addData("Current Configuration", currentConfiguration + " - " + autoSwitcher.get(currentConfiguration));
            telemetry.addData("Start Location", autoSwitcher.get(START_LOCATION));
            telemetry.addData("Delay Seconds", autoSwitcher.get(DELAY_SECONDS));
            telemetry.addData("Place Location", autoSwitcher.get(PLACE_LOCATION));
            telemetry.addData("Park Location", autoSwitcher.get(PARK_LOCATION));
            telemetry.addData("Movement Path", autoSwitcher.get(MOVEMENT_PATH));
            telemetry.addData("Prop Guess", propGuess);
            telemetry.addLine()
                    .addData("X", "%.2f", holOdom.getPose().getX())
                    .addData("Y", "%.2f", holOdom.getPose().getY())
                    .addData("Heading", "%.3f", holOdom.getPose().getHeading());
            telemetry.addLine("Status Detail:")
                    .addData("Viper Zeroed", true)
                    .addData("Arm Pickup", systemStates[0])
                    .addData("Vision Target Acquired", systemStates[1])
                    .addData("Arm Up", systemStates[2]);
            telemetry.update();
        }

        waitForStart();

        double timer = System.currentTimeMillis();

        robot.cameraSubsystem.setColorBlobDetector(null);
        robot.pixelSubsystem.requestPD();

        ColorBlobDetector.PropGuess mirroredGuess = propGuess;
        if (autoSwitcher.getAlliance() == NewAutoSwitcher.Alliance.RED) {
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
            case RED_BACKDROP: case BLUE_BACKDROP:
                //Backdrop side code
                if(autoSwitcher.getDelayMS() != 0) {
                    sleep(autoSwitcher.getDelayMS());
                }
                switch (mirroredGuess) {
                    case LEFT:
                        autoSwitcher.moveSwitch(22, 48, -1.57, 1);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(18, 34, 3.14, 1);
                        autoSwitcher.moveSwitch(15, 34, 3.14, 1);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(16, 35.5, -1.57, 1);
                        break;
                }

                while(!robot.pixelSubsystem.isPDReady() && opModeIsActive()) {
                    robot.pixelSubsystem.runPixelSystem();
                    telemetry.addData("Lift Postion", leftLift.getCurrentPosition());
                    telemetry.update();
                }

                robot.pixelSubsystem.setFingers(OPEN, CLOSED);

                sleep(300);

                robot.pixelSubsystem.endPDresumeND();


                if (mirroredGuess == ColorBlobDetector.PropGuess.LEFT) {
                    autoSwitcher.moveSwitch(30, 55, -1.57, 1, false);
                } else if(mirroredGuess == ColorBlobDetector.PropGuess.MIDDLE || mirroredGuess == ColorBlobDetector.PropGuess.UNKNOWN) {
                    autoSwitcher.moveSwitch(16, 50, -1.57, 1, false);
                }
                break;
            case BLUE_AUDIENCE: case RED_AUDIENCE:
                //Place Purple Pixel
                switch (mirroredGuess) {
                    case LEFT:
                        autoSwitcher.moveSwitch(-18, 34, 3.14, 1);
                        autoSwitcher.moveSwitch(-15, 34, 3.14, 1);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(-22, 48, -1.57, 1);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(16, 35.5, -1.57, 1);
                        break;
                }

                while(!robot.pixelSubsystem.isPDReady() && opModeIsActive()) {
                    robot.pixelSubsystem.runPixelSystem();
                }

                robot.pixelSubsystem.setFingers(OPEN, CLOSED);

                sleep(300);

                robot.pixelSubsystem.returnArm(true);
                double timestamp = System.currentTimeMillis();
                //Grab one white pixel from stack
                autoSwitcher.moveSwitch(-55, 38, 0, 1, false);
                while (System.currentTimeMillis() - timestamp < 2000 && opModeIsActive()) {
                    Pose2d pose2d = robot.cameraSubsystem.getPoseFromAprilTag();
                    if (pose2d != null) {
                        holOdom.updatePose(pose2d);
                        Pose2d firstOutput = holOdom.getPose();
                        robot.resetOdo(pose2d);
                        holOdom.updatePose();
                        Pose2d secondOutput = holOdom.getPose();

                        telemetry.addData("April Input X", pose2d.getX())
                                .addData("Y", pose2d.getY())
                                .addData("Heading", pose2d.getHeading());
                        telemetry.addData("First Output X", firstOutput.getX())
                                .addData("Y", firstOutput.getY())
                                .addData("Heading", firstOutput.getHeading());
                        telemetry.addData("Second Output X", secondOutput.getX())
                                .addData("Y", secondOutput.getY())
                                .addData("Heading", secondOutput.getHeading());
                        telemetry.addData("April Lock Time", System.currentTimeMillis() - timestamp);
                        telemetry.update();
                        break;
                    }
                }
                //Recenter using apriltag
//                robot.movementSubsystem.requestApriltagSync();

                //Move to ready spot

                robot.pixelSubsystem.toggleIntake();
                robot.pixelSubsystem.frontStageOverride(5);

                //Move to grab
                autoSwitcher.moveSwitch(-55.5, 36, 0, 1, true);
                double t = System.currentTimeMillis();
                while (opModeIsActive()) {
                    if((System.currentTimeMillis() - t > 2000 && robot.pixelSubsystem.pixelCount > 0) || System.currentTimeMillis() - t > 4000) {
                        robot.pixelSubsystem.toggleIntake();
                        break;
                    }
                    robot.pixelSubsystem.runPixelSystem();
                    if (robot.pixelSubsystem.pixelCount >= 2) {
                        break;
                    }
                    autoSwitcher.moveSwitch(-55.5, 36, 0, 1, true);
                    if (robot.pixelSubsystem.pixelCount >= 2) {
                        break;
                    }
                    autoSwitcher.moveSwitch(-56.75, 36, 0, 1, true);
                }

                //Wait until pixel is detected

                robot.pixelSubsystem.toggleIntake();
                robot.pixelSubsystem.fingerOverrideBase(true);

                //Go to Backdrop
                requestOpModeStop();

                break;
        }

        //Merged Code
//        robot.movementSubsystem.requestApriltagSync();
        autoSwitcher.moveSwitch(36, 36, 0, 1, false);
        robot.pixelSubsystem.liftSet(1100);

        PlaceLocation placeMirrored = autoSwitcher.getPlaceLocation();
        if (autoSwitcher.getAlliance() == NewAutoSwitcher.Alliance.RED) {
            switch (placeMirrored) {
                case LEFT:
                    placeMirrored = NewAutoSwitcher.PlaceLocation.RIGHT;
                    break;
                case RIGHT:
                    placeMirrored = LEFT;
                    break;
            }
        }

        final double LEFT_TO_RIGHT = placeMirrored == LEFT ? 0 : 2;

        //Depth Control
        double backdropDistance = distance.getDistance(DistanceUnit.INCH);
        double dropX = holOdom.getPose().getX() + backdropDistance - 2.7;


        //Drop Pixel on Backdrop. Default points to LEFT edge of the drop zone.
        switch (mirroredGuess) {
            case LEFT:
                autoSwitcher.moveSwitch(dropX, 41, 0, 0.5);
                break;
            case RIGHT:
                autoSwitcher.moveSwitch(dropX, 35.5, 0, 0.5);
                break;
            case MIDDLE:
            default:
                autoSwitcher.moveSwitch(dropX, 38.5, 0, 0.5);
        }
        robot.pixelSubsystem.flipHorizontal(true);
        robot.pixelSubsystem.runPixelSystem();
        sleep(700);
        robot.pixelSubsystem.dropRight(true);
        robot.pixelSubsystem.runPixelSystem();
        sleep(700);

        autoSwitcher.moveSwitch(42, 36, 0, 1, false);

        boolean skipCycle;

        skipCycle = (30 * 1000) - (System.currentTimeMillis() - timer) < 19000;

        //Cycling Loop
        int cycles = 1;
        for (int i = 0; i < cycles; i++) {
            if(!opModeIsActive() || skipCycle) break;

            robot.pixelSubsystem.returnArm(true);
            double yMovement = i == 0 ? 17 : 12;
            switch (autoSwitcher.getMovementPath()) {
                case OUTSIDE_TRUSS: case INSIDE_TRUSS:
                    autoSwitcher.moveSwitch(42, yMovement, 0, 1, false);
                    autoSwitcher.moveSwitch(-24, yMovement, 0, 1, false);
                    autoSwitcher.moveSwitch(-24, yMovement, Math.toRadians(-40), 1, false);
                    break;
                case STAGE_DOOR:
                    autoSwitcher.moveSwitch(42, 58,0,1,false);
                    autoSwitcher.moveSwitch(-24, 58,0,1,false);
                    autoSwitcher.moveSwitch(-24, 58,Math.toRadians(40),1,false);
                    break;
            }

            double timestamp = System.currentTimeMillis();
            telemetry.addData("Camera Status", "Looking");
            telemetry.update();
            while (System.currentTimeMillis() - timestamp < 2000 && opModeIsActive()) {
                Pose2d pose2d = robot.cameraSubsystem.getPoseFromAprilTag();
                if (pose2d != null) {
                    holOdom.updatePose(pose2d);
                    Pose2d firstOutput = holOdom.getPose();
                    robot.resetOdo(pose2d);
                    holOdom.updatePose();
                    Pose2d secondOutput = holOdom.getPose();

                    telemetry.addData("April Input X", pose2d.getX())
                            .addData("Y", pose2d.getY())
                            .addData("Heading", pose2d.getHeading());
                    telemetry.addData("First Output X", firstOutput.getX())
                            .addData("Y", firstOutput.getY())
                            .addData("Heading", firstOutput.getHeading());
                    telemetry.addData("Second Output X", secondOutput.getX())
                            .addData("Y", secondOutput.getY())
                            .addData("Heading", secondOutput.getHeading());
                    telemetry.addData("April Lock Time", System.currentTimeMillis() - timestamp);
                    telemetry.update();
                    break;
                }
            }

            double yStack = 0;
            switch (autoSwitcher.getMovementPath()) {
                case OUTSIDE_TRUSS: case INSIDE_TRUSS:
                    yStack = 34;
                    break;
                case STAGE_DOOR:
                    yStack = 10.75;
                    break;
            }

            autoSwitcher.moveSwitch(-52, yStack, 0, 1, true);
            robot.pixelSubsystem.toggleIntake();
            robot.pixelSubsystem.frontStageOverride(5 - i);
            autoSwitcher.moveSwitch(-56.5, yStack, 0, 1, true);
            robot.pixelSubsystem.frontStageOverride(4 - i);
            double timestamo = System.currentTimeMillis();
            while (opModeIsActive()) {
                if((System.currentTimeMillis() - timestamo > 2000 && robot.pixelSubsystem.pixelCount > 0) || System.currentTimeMillis() - timestamo > 4000) {
                    robot.pixelSubsystem.toggleIntake();
                    break;
                }
                robot.pixelSubsystem.runPixelSystem();
                if (robot.pixelSubsystem.pixelCount >= 2) {
                    break;
                }
                autoSwitcher.moveSwitch(-55.5, yStack, 0, 1, true);
                if (robot.pixelSubsystem.pixelCount >= 2) {
                    break;
                }
                autoSwitcher.moveSwitch(-56.75, yStack, 0, 1, true);
            }
            robot.pixelSubsystem.setFingers(CLOSED, CLOSED);
            robot.pixelSubsystem.runPixelSystem();
            switch (autoSwitcher.getMovementPath()) {

                case INSIDE_TRUSS: case OUTSIDE_TRUSS:
                    autoSwitcher.moveSwitch(-48, 55, 0, 1, true);
                    autoSwitcher.moveSwitch(40, 55, 0, 1, true);
                    break;
                case STAGE_DOOR:
                    autoSwitcher.moveSwitch(40, 10, 0, 1);
                    break;
            }


            robot.pixelSubsystem.liftSet(1275);
            autoSwitcher.moveSwitch(40, 38, 0, 1);
            backdropDistance = distance.getDistance(DistanceUnit.INCH);
            dropX = holOdom.getPose().getX() + backdropDistance - 2.7;
            autoSwitcher.moveSwitch(dropX, 38, 0, 1);

            robot.pixelSubsystem.dropRight(true);
            robot.pixelSubsystem.runPixelSystem();

            timestamo = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - timestamo < 500) {
                robot.pixelSubsystem.runPixelSystem();
            }
            robot.pixelSubsystem.returnArm(true);


        }
        autoSwitcher.moveSwitch(45, 38, 0, 1);
        switch (autoSwitcher.getParkLocation()) {
            case INSIDE:
                autoSwitcher.moveSwitch(46, 15, 0, 1);
                break;
            case OUTSIDE:
                autoSwitcher.moveSwitch(46, 60.5, 0, 1);
                break;
        }

//        telemetry.addData("Runtime", "%.2f", (System.currentTimeMillis() - timer) / 1000);
//        telemetry.update();

        while (opModeIsActive()) {
            robot.pixelSubsystem.runPixelSystem();
        }
    }
}
