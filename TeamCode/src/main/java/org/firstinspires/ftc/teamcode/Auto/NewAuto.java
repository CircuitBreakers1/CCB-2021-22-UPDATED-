package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.DELAY_SECONDS;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.MOVEMENT_PATH;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.PARK_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.PLACE_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.ConfigSetting.START_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.PlaceLocation;
import static org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher.PlaceLocation.LEFT;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector;
import org.firstinspires.ftc.teamcode.Subsystems.NewAutoSwitcher;
import org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023;

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
                        robot.pixelSubsystem.liftSet(200);
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

            propGuess = ColorBlobDetector.PropGuess.MIDDLE; //robot.cameraSubsystem.getPropGuess();
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

        //robot.cameraSubsystem.setColorBlobDetector(null);
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
                        autoSwitcher.moveSwitch(22, 55, -1.57, 1);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(15, 48.5, -0.685, 1);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(16, 46, -1.57, 1);
                        break;
                }

                while(!robot.pixelSubsystem.isPDReady() && opModeIsActive()) {
                    robot.pixelSubsystem.runPixelSystem();
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
                break;
        }

        //Merged Code
        autoSwitcher.moveSwitch(36, 36, -3.14, 1, false);

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

        //Drop Pixel on Backdrop. Default points to LEFT edge of the drop zone.
        switch (mirroredGuess) {
            case LEFT:
                autoSwitcher.moveSwitch(50.2, 44 - LEFT_TO_RIGHT, -3.14, 0.5);
                break;
            case RIGHT:
                autoSwitcher.moveSwitch(50.2, 33 - LEFT_TO_RIGHT, -3.14, 0.5);
                break;
            case MIDDLE:
            default:
                autoSwitcher.moveSwitch(50.2, 39 - LEFT_TO_RIGHT, -3.14, 0.5);
        }

        robot.pixelSubsystem.dropRight(true);
        sleep(300);
        autoSwitcher.moveSwitch(36, 36, -3.14, 1, false);
        robot.pixelSubsystem.returnArm(true);

        switch (autoSwitcher.getParkLocation()) {
            case INSIDE:
                autoSwitcher.moveSwitch(36, 15, -3.14, 1);
                autoSwitcher.moveSwitch(46, 15, -3.14, 1);
                break;
            case OUTSIDE:
                autoSwitcher.moveSwitch(36, 60.5, -3.14, 1);
                autoSwitcher.moveSwitch(46, 60.5, -3.14, 1);
                break;
        }
    }
}
