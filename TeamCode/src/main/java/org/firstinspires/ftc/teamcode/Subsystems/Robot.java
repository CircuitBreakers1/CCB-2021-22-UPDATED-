package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Subsystems.fieldSquares.*;
import static org.firstinspires.ftc.teamcode.Subsystems.pathType.*;
import static java.lang.Math.*;


/**
 * This class contains all hardware maps and functions for use in opModes. An object should be
 * created, then during init phase {@link #init init} should be called with the opMode hwmap.
 *
 * This class also creates a basic framework of telemetry. It sets auto clear to off, meaning that
 * the first time something is intended to be added to telemetry it should be created using
 * .addData() and stored in a Telemetry.Item. To update, use .setValue() with the Item.
 */
public class Robot {
    public static MotorEx leftFront;
    public static MotorEx rightFront;
    public static MotorEx leftBack;
    public static MotorEx rightBack;

    public static CRServo pickupLeft;
    public static CRServo pickupRight;

    public static DcMotor armLift;

    public static DigitalChannel coneTouch;

    public static MotorEx leftOdo;
    public static MotorEx rightOdo;
    public static MotorEx backOdo;

    public static BNO055IMU imu;

    public static float xLoc = 0;
    public static float yLoc = 0;
    public static float rotation = 0;

    private static char squareXLocation = 'A';
    private static int squareYLocation = 1;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private static OpMode opMode;
    private static boolean opIsAuto;
    private final ElapsedTime period  = new ElapsedTime();

    //All distance values in inches
    private static final float wheelRadius = (float) 1.49606 / 2;
    private static final float ticksPerRev = 1440;
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    private static final double TRACKWIDTH = 12;
    private static final double CENTER_WHEEL_OFFSET = -4.9375; //Was 7.75



    //Create the Subsystems
    public static DrivetrainSubsystem drivetrain;
    public static HolonomicOdometry holOdom = new HolonomicOdometry(
            () -> leftOdo.getCurrentPosition() * ticksToIn,
            () -> rightOdo.getCurrentPosition() * ticksToIn,
            () -> backOdo.getCurrentPosition() * ticksToIn,
            TRACKWIDTH, CENTER_WHEEL_OFFSET
    );
    public static PositionalMovementSubsystem positionalMovement;


    /**
     * Initalize the robot object.
     * @param OPMode <b>Always</b> pass <i>this</i> here as it allows the telemetry to initialize.
     * @param isAuto Tells the robot whether or not to add telemetry for autonomous
     */
    public Robot(OpMode OPMode, boolean isAuto){
        //Grab opmode object from active opMode for telemetry
        opMode = OPMode;
        opIsAuto = isAuto;
    }

    private void initHelp(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = new MotorEx(hwMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hwMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hwMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hwMap, "rightBack", Motor.GoBILDA.RPM_312);
        armLift = hwMap.get(DcMotor.class, "armLift");

        pickupLeft = hwMap.get(CRServo.class, "pickupLeft");
        pickupRight = hwMap.get(CRServo.class, "pickupRight");

        coneTouch = hwMap.get(DigitalChannel.class, "coneTouch");

        leftOdo = new MotorEx(hwMap, "leftOdo");
        rightOdo = new MotorEx(hwMap, "rightOdo");
        backOdo = new MotorEx(hwMap, "backOdo");
        /*
        if(opIsAuto) {
            leftBack.setRunMode(Motor.RunMode.VelocityControl);
            leftFront.setRunMode(Motor.RunMode.VelocityControl);
            rightBack.setRunMode(Motor.RunMode.VelocityControl);
            rightFront.setRunMode(Motor.RunMode.VelocityControl);
        } else { */
            leftBack.setRunMode(Motor.RunMode.RawPower);
            leftFront.setRunMode(Motor.RunMode.RawPower);
            rightBack.setRunMode(Motor.RunMode.RawPower);
            rightFront.setRunMode(Motor.RunMode.RawPower);



        coneTouch.setMode(DigitalChannel.Mode.INPUT);

        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        */

        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBack.setInverted(true);
        leftFront.setInverted(true);

        leftOdo.setDistancePerPulse(ticksToIn);
        rightOdo.setDistancePerPulse(ticksToIn);
        backOdo.setDistancePerPulse(ticksToIn);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rightBack.resetEncoder();
        rightFront.resetEncoder();
        leftBack.resetEncoder();
        leftFront.resetEncoder();

        rightOdo.resetEncoder();
        leftOdo.resetEncoder();
        backOdo.resetEncoder();


        leftOdo.setInverted(true);
        rightOdo.setInverted(false);
        backOdo.setInverted(false);


        //ButtonToggleSubsystem.clearList();
    }

    public void init(HardwareMap ahwMap) {
        initHelp(ahwMap);

        holOdom.updatePose(new Pose2d());

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();

        drivetrain = new DrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
        positionalMovement = new PositionalMovementSubsystem(drivetrain, holOdom, opMode);
    }

    public void init(HardwareMap ahwMap, double startX, double startY, double startDegrees) {
        initHelp(ahwMap);

        Pose2d startPose = new Pose2d(startX, startY, Rotation2d.fromDegrees(startDegrees));

        holOdom.updatePose(startPose);

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();

        drivetrain = new DrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
        positionalMovement = new PositionalMovementSubsystem(drivetrain, holOdom, opMode);
    }






}
