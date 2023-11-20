package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.BACKMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.LEFTMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.OFFSETMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.RIGHTMULT;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * The main class for the robot. Controls everything, however larger and
 * more complex functionality is passed off into different subsystems
 */
public class Robot2023 {
    //All values in IN
    public static final float wheelRadius = (float) 1.37795 / 2;
    public static final float ticksPerRev = 1440;
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    /**
     * Max acceptable deviation when moving using odometry. Functions as the radius of a circle
     */

    public static MotorEx leftFront;
    public static MotorEx leftBack;
    public static MotorEx rightFront;
    public static MotorEx rightBack;

    public static DcMotor intake;
    public static DcMotor lift;
    public static DcMotor armAngle;
    public static DcMotor armExtend;

    public static MotorEx leftOdo;
    public static MotorEx rightOdo;
    public static MotorEx frontOdo;

    public static AnalogInput armAngleEncoder;

    public static DigitalChannel viperTouch;
    public static NormalizedColorSensor leftBay;
    public static NormalizedColorSensor rightBay;
    public static Servo liftRaise;
    public static Servo wrist;
    public static Servo shotRelease;
    public static Servo gripper;
    public static Servo slidePush;
    public static Servo shooterRaise;
    public static IMU imu;

    private boolean visionInit = false;
    private boolean startLocationBasedOnApril = false;
    private double recalibrateTime = 0;

    public static final String LOGCATTAG = "Robot Logging: ";

    //Tuning Values
    float colorGain = 2;
    double trackwidth = 14.14;
    double odoOffset = 6.5515;

    //Subsystems
    public HoloDrivetrainSubsystem holoDrivetrain;
    public static HolonomicOdometry holOdom;
    public MovementSubsystem movementSubsystem;
    public ArmSubsystem armSubsystem;
    public CameraSubsystem cameraSubsystem;
    /**
     *
     * @param s The minimum time between recalibrating position based on AprilTag.
     *          Once the time has elapsed, the robot will reopen the camera stream, and attempt
     *          to recalibrate its position. Once it has done so, it will reclose the camera stream.
     */
    public Robot2023(double s) {
        recalibrateTime = s;
    }

    public Robot2023() {
        recalibrateTime = -1;
    }

    public void init(HardwareMap ahwMap, boolean initVision, LinearOpMode opMode) {
        leftFront = new MotorEx(ahwMap, "leftFront");
        leftBack = new MotorEx(ahwMap, "leftBack");
        rightFront = new MotorEx(ahwMap, "rightFront");
        rightBack = new MotorEx(ahwMap, "rightBack");

        intake = ahwMap.get(DcMotor.class, "intake");
        lift = ahwMap.get(DcMotor.class, "lift");
        armAngle = ahwMap.get(DcMotor.class, "armAngle");
        armExtend = ahwMap.get(DcMotor.class, "armExtend");

        //Left is on armAngle, Right is on intake, Front is on lift
        leftOdo = new MotorEx(ahwMap, "armAngle");
        rightOdo = new MotorEx(ahwMap, "intake");
        frontOdo = new MotorEx(ahwMap, "lift");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRaise = ahwMap.get(Servo.class, "liftRaise");
        wrist = ahwMap.get(Servo.class, "wrist");
        shotRelease = ahwMap.get(Servo.class, "shotRelease");
        gripper = ahwMap.get(Servo.class, "gripper");
        slidePush = ahwMap.get(Servo.class, "slidePush");
        shooterRaise = ahwMap.get(Servo.class, "shooterRaise");

        imu = ahwMap.get(IMU.class, "imu");

        rightBay = ahwMap.get(NormalizedColorSensor.class, "rightBay");
        leftBay = ahwMap.get(NormalizedColorSensor.class, "leftBay");

        rightBay.setGain(colorGain);
        leftBay.setGain(colorGain);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        armAngleEncoder = ahwMap.get(AnalogInput.class, "armAngleEncoder");

        viperTouch = ahwMap.get(DigitalChannel.class, "viperTouch");

        rightFront.setInverted(true);
        rightBack.setInverted(true);

        leftBack.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        leftBack.setRunMode(MotorEx.RunMode.RawPower);
        leftFront.setRunMode(MotorEx.RunMode.RawPower);
        rightBack.setRunMode(MotorEx.RunMode.RawPower);
        rightFront.setRunMode(MotorEx.RunMode.RawPower);

        leftOdo.setDistancePerPulse(ticksToIn);
        rightOdo.setDistancePerPulse(ticksToIn);
        frontOdo.setDistancePerPulse(ticksToIn);

        leftOdo.resetEncoder();
        rightOdo.resetEncoder();
        frontOdo.resetEncoder();

        holOdom = new HolonomicOdometry(
                () -> (leftOdo.getCurrentPosition() * ticksToIn * LEFTMULT),
                () -> rightOdo.getCurrentPosition() * -ticksToIn * RIGHTMULT,
                () -> frontOdo.getCurrentPosition() * ticksToIn * BACKMULT,
                trackwidth,
                odoOffset * OFFSETMULT
        );

        holOdom.updatePose();
        //The front of the robot from the drivers control perspective is the lift side, but for odometry it is the intake side
        holOdom.updatePose(new Pose2d(0,0, new Rotation2d(3.14)));

        holoDrivetrain = new HoloDrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
        if(opMode != null) movementSubsystem = new MovementSubsystem(holoDrivetrain, holOdom, opMode, cameraSubsystem, imu);
        armSubsystem = new ArmSubsystem(wrist, gripper, armAngle, armAngleEncoder, armExtend);

        if(initVision) {
            cameraSubsystem = new CameraSubsystem(ahwMap.get(WebcamName.class, "Webcam"), ColorBlobDetector.PropColor.BLUE);
            visionInit = true;
        }
    }

    public boolean isVisionInit() {
        return visionInit;
    }
}
