package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * The main class for the robot. Controls everything, however larger and
 * more complex functionality is passed off into different subsystems
 */
public class NewRobot2023 {


    //Motors
    public static MotorEx leftFront;
    public static MotorEx leftBack;
    public static MotorEx rightFront;
    public static MotorEx rightBack;
    public static DcMotor leftLift;
    public static DcMotor rightLift;
    public static DcMotor intake;
    public static MotorEx leftOdo;
    public static MotorEx rightOdo;
    public static MotorEx frontOdo;

    //Servos
    public static Servo through;
    public static Servo rotate;
    public static Servo frontStage;
    public static Servo leftFinger;
    public static Servo rightFinger;
    public static Servo shoot;


    //Sensors
    public static IMU imu;
    public static NormalizedColorSensor colorSensor;
    public static DigitalChannel touch;
    public static DistanceSensor distance;

    //Subsystems
    public HoloDrivetrainSubsystem holoDrivetrain;
    public static HolonomicOdometry holOdom;
    public NewMovementSubsystem movementSubsystem;
    public CameraSubsystem cameraSubsystem;
    public PixelSubsystem pixelSubsystem;

    //Tuning Values - All values in inches unless noted
    float colorGain = 2; //Units: None
    double trackwidth = 11.8705;
    double odoOffset = 1.1705;
    public static final float wheelRadius = (float) 1.37795 / 2;
    public static final float ticksPerRev = 1440; //Units: Ticks/Rev
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev); //Units: In/Ticks


    public NewRobot2023() {}

    public void init(HardwareMap ahwMap, boolean initVision, LinearOpMode opMode) {
        //Init Motors
        {
            leftFront = new MotorEx(ahwMap, "leftFront");
            leftBack = new MotorEx(ahwMap, "leftBack");
            rightFront = new MotorEx(ahwMap, "rightFront");
            rightBack = new MotorEx(ahwMap, "rightBack");

            leftLift = ahwMap.get(DcMotor.class, "leftLift");
            rightLift = ahwMap.get(DcMotor.class, "rightLift");
            intake = ahwMap.get(DcMotor.class, "intake");

            leftOdo = new MotorEx(ahwMap, "leftFront");
            rightOdo = new MotorEx(ahwMap, "leftBack");
            frontOdo = new MotorEx(ahwMap, "rightFront");

            leftFront.setInverted(true);
            rightBack.setInverted(true);

            rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftOdo.setDistancePerPulse(ticksToIn);
            rightOdo.setDistancePerPulse(ticksToIn);
            frontOdo.setDistancePerPulse(ticksToIn);

            leftOdo.resetEncoder();
            rightOdo.resetEncoder();
            frontOdo.resetEncoder();
        }

        //Init Servos
        {
            rotate = ahwMap.get(Servo.class, "rotate");
            through = ahwMap.get(Servo.class, "through");
            frontStage = ahwMap.get(Servo.class, "frontStage");
            leftFinger = ahwMap.get(Servo.class, "leftFinger");
            rightFinger = ahwMap.get(Servo.class, "rightFinger");
            shoot = ahwMap.get(Servo.class, "shoot");
        }


        //Init Sensors
        {
//            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
//            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//            imu.initialize(new IMU.Parameters(orientationOnRobot));

            colorSensor = ahwMap.get(NormalizedColorSensor.class, "colorSensor");
            touch = ahwMap.get(DigitalChannel.class, "touch");
            distance = ahwMap.get(DistanceSensor.class, "distance");
        }

        //Init Subsystems
        {
            holoDrivetrain = new HoloDrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
            holOdom = new HolonomicOdometry(
                    () -> leftOdo.getCurrentPosition() * ticksToIn,
                    () -> rightOdo.getCurrentPosition() * ticksToIn,
                    () -> frontOdo.getCurrentPosition() * ticksToIn,
                    -trackwidth,
                    odoOffset
            );

            holOdom.updatePose();
            holOdom.updatePose(new Pose2d(0, 0, new Rotation2d(0)));

            pixelSubsystem = new PixelSubsystem(leftLift, rightLift, intake, frontStage, through, rotate, leftFinger, rightFinger, touch, colorSensor);

            if(initVision) {
                cameraSubsystem = new CameraSubsystem(ahwMap.get(WebcamName.class, "Webcam"), ahwMap.get(WebcamName.class, "Webcam2"));
            }

            if(opMode != null) {
                movementSubsystem = new NewMovementSubsystem(holoDrivetrain, holOdom, opMode, cameraSubsystem, pixelSubsystem);
            }
        }

    }

    public void resetOdo(Pose2d pose) {
        leftOdo.resetEncoder();
        rightOdo.resetEncoder();
        frontOdo.resetEncoder();
        holOdom = new HolonomicOdometry(
                () -> leftOdo.getCurrentPosition() * ticksToIn,
                () -> rightOdo.getCurrentPosition() * ticksToIn,
                () -> frontOdo.getCurrentPosition() * ticksToIn,
                -trackwidth,
                odoOffset
        );

        holOdom.updatePose();
        holOdom.updatePose(pose);
    }

    public void shoot() {
        shoot.setPosition(0.2);
    }

    public void resetShoot() {
        shoot.setPosition(0);
    }
}
