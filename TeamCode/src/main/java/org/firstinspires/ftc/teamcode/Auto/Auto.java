package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.ConfigSetting.START_LOCATION;
import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.StartLocation.BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.StartLocation.BLUE_BACKDROP;
import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.StartLocation.RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.StartLocation.RED_BACKDROP;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.base;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.holOdom;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.viperTouch;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.wrist;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.t1;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.t2;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.t3;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.x1;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.x2;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.x3;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.y1;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.y2;
import static org.firstinspires.ftc.teamcode.Tuning.AutoTuning.y3;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSwitcher.ConfigSetting;
import org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector;
import org.firstinspires.ftc.teamcode.Subsystems.ColorDetectionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MovementSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PoseSupply;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@Autonomous(name = "Auto", group = "Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 robot = new Robot2023();

        robot.init(hardwareMap, false, this);

        AutoSwitcher autoSwitcher = new AutoSwitcher(robot.movementSubsystem);
        ConfigSetting currentConfiguration = START_LOCATION;

        ColorBlobDetector.PropColor currentColor = ColorBlobDetector.PropColor.BLUE;
        robot.cameraSubsystem.setColorBlobDetector(currentColor);

        ColorBlobDetector.PropGuess propGuess = ColorBlobDetector.PropGuess.UNKNOWN;

        ArmSubsystem.ArmState armState = ArmSubsystem.ArmState.Ready;

        //Represents whether robot states are ready:
        //Indexes: 0 - Arm Pickup Maneuver Happened, 1 - Right Bay Shows YELLOW, 2 - Vision Target Acquired, 3 - Odo not in Timbucktoo, 4 - Arm Length Zeroed, 5 - Arm Angle Zeroed
        boolean[] systemStates = new boolean[6];
        boolean allSystemsGo = false;

        double gripTime = 0;

        //Up, right, down, left
        boolean[] dpadPressed = {false, false, false, false};

        //TODO: Implement Lights to Show Ready Status
        while(opModeInInit()) {
            //Do Arm Zeroing
            if(!(abs(robot.armSubsystem.getAngle() - 7) < 4 /*Angle not in position*/ && !systemStates[5])) {
                armAngle.setPower(-0.8 * Math.signum(robot.armSubsystem.getAngle() - 7));
            } else {
                armAngle.setPower(0);
                systemStates[5] = true;
            }

            if(!systemStates[4]) {
                if(!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */ ) {
                    armExtend.setPower(0);
                    armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    systemStates[4] = true;
                }
                armExtend.setPower(0.25);
            } else {
                armExtend.setTargetPosition(base);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(0.8);
            }

            //Perform Arm Maneuver
            switch (armState) {
                case Ready:
                    gripper.setPosition(0);
                    robot.armSubsystem.setWristAngle(0);
                    if(gamepad1.a && systemStates[4] && systemStates[5]) {
                        armState = ArmSubsystem.ArmState.LowerArm;
                    }
                    break;
                case LowerArm:
                    gripper.setPosition(1);
                    armAngle.setPower(-0.85);
                    if(abs(robot.armSubsystem.getAngle() - 2) < 1) {
                        armAngle.setPower(0);
                        armState = ArmSubsystem.ArmState.Grip;
                    }
                    break;
                case Grip:
                    if(gripTime == 0) {
                        gripTime = System.currentTimeMillis();
                    }
                    gripper.setPosition(0);
                    if(System.currentTimeMillis() - gripTime > 500) {
                        armState = ArmSubsystem.ArmState.RaiseArm;
                        gripTime = 0;
                    }
                    break;
                case RaiseArm:
                    armAngle.setPower(0.85);
                    if(robot.armSubsystem.getAngle() > 0 /*Arm is raised*/) {
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
                if(!dpadPressed[0]) {
                    dpadPressed[0] = true;
                    autoSwitcher.toggleUp(currentConfiguration);
                    if(currentConfiguration == START_LOCATION) {
                        switch (autoSwitcher.getStartLocation()) {
                            case RED_BACKDROP:
                            case RED_AUDIENCE:
                                currentColor = ColorBlobDetector.PropColor.RED;
                                break;
                            case BLUE_AUDIENCE:
                            case BLUE_BACKDROP:
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
                if(!dpadPressed[2]) {
                    dpadPressed[2] = true;
                    autoSwitcher.toggleDown(currentConfiguration);
                    if(currentConfiguration == START_LOCATION) {
                        switch (autoSwitcher.get(START_LOCATION)) {
                            case "RED_BACKDROP":
                            case "RED_AUDIENCE":
                                currentColor = ColorBlobDetector.PropColor.RED;
                                break;
                            case "BLUE_BACKDROP":
                            case "BLUE_AUDIENCE":
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
                if(!dpadPressed[3]) {
                    dpadPressed[3] = true;
                    currentConfiguration = ConfigSetting.cycleDown(currentConfiguration);
                }
            } else {
                dpadPressed[3] = false;
            }
            if (gamepad1.dpad_right) {
                if(!dpadPressed[1]) {
                    dpadPressed[1] = true;
                    currentConfiguration = ConfigSetting.cycleUp(currentConfiguration);
                }
            } else {
                dpadPressed[1] = false;
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

            telemetry.addLine("Status: " + (allSystemsGo ? "ALL SYSTEMS GO!" : "Not Ready"));
            telemetry.addData("Current Configuration", currentConfiguration + " - " + autoSwitcher.get(currentConfiguration));
            telemetry.addData("Start Location", autoSwitcher.get(START_LOCATION));
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
        robot.cameraSubsystem.setColorBlobDetector(null);
        armExtend.setTargetPosition(-600);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setPower(0.8);
        //Create the runnable to lower the arm as we move to the spike marks.
        Runnable lowerArm = () -> {
            //Make sure the arm is far enough out
            if(armExtend.getCurrentPosition() < -200) {
                double error = -20 - robot.armSubsystem.getAngle();
                if(abs(error) > 3) {
                    armAngle.setPower(0.85 * signum(error));
                }
            }
        };

        switch (autoSwitcher.getStartLocation()) {
            case RED_BACKDROP:
            case BLUE_BACKDROP:
                //Backdrop side code
                switch (propGuess) {
                    case LEFT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x1, y1, t1, 1, lowerArm);
                        break;
                    case RIGHT:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x2, y2, t2, 1, lowerArm);
                        break;
                    case MIDDLE:
                    case UNKNOWN:
                        autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, x3, y3, t3, 1, lowerArm);
                        break;
                }
                //Finish moving the arm, then stop the arm and drop the pixel.
                while(opModeIsActive()) {
                    if(armExtend.getCurrentPosition() < -200) {
                        double error = -20 - robot.armSubsystem.getAngle();
                        if(abs(error) > 3) {
                            armAngle.setPower(0.85 * signum(error));
                        } else {
                            break;
                        }
                    }
                }
                armAngle.setPower(0);
                //Drop the pixel.
                gripper.setPosition(1);
                sleep(300);
                //Go to the backboard, then the code merge happens
                //autoSwitcher.moveSwitch(PoseSupply.ODOMETRY, 0,0,0, 1, MovementSubsystem.GRAB_PIXEL_AUTO);
                break;
            case RED_AUDIENCE:
            case BLUE_AUDIENCE:

                break;
        }
        //Once we get to the backdrop, the code merges.
        Runnable placePixelAnglePrep = () -> {
            double error = 30 - robot.armSubsystem.getAngle();
            if(abs(error) > 3) {
                armAngle.setPower(0.85 * signum(error));
            }
        };

    }
}
