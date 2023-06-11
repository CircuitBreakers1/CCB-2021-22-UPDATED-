package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.APRIL_SLEEVE;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.COLOR_SLEEVE;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.CONE_STACK;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.ONE_CAM_JUNCTION;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.Robot.cameraInit.TWO_CAM_JUNCTION;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.tuningConstants.trackTuning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 * This class contains all hardware maps and functions for use in opModes. An object should be
 * created, then during init phase {@link #init init} should be called with the opMode hwmap.
 * <p>
 * This class also creates a basic framework of telemetry. It sets auto clear to off, meaning that
 * the first time something is intended to be added to telemetry it should be created using
 * .addData() and stored in a Telemetry.Item. To update, use .setValue() with the Item.
 */
public class Robot {
    public static DcMotorEx leftFront;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftBack;
    public static DcMotorEx rightBack;

    public static RevBlinkinLedDriver blinkinLedDriver;
    public static RevBlinkinLedDriver.BlinkinPattern pattern;

    public static CRServo pickupLeft;
    public static CRServo pickupRight;

    public static DcMotor armLift;

    public static DigitalChannel coneTouch;
    public static DigitalChannel armTouch;

    public static OpenCvCamera camera;
    public static OpenCvCamera camera2;

    private static cameraInit cameraInit;
    public static ColorSleevePipeline colorSleevePipeline = new ColorSleevePipeline();
    public static ApriltagSleevePipeline apriltagSleevePipeline =
            new ApriltagSleevePipeline(0.0254, 578.272, 578.272, 402.145, 221.506);

    public static ColorJunctionDetectionPipeline colorJunctionLeftPipeline =
            new ColorJunctionDetectionPipeline(true);

    public static ColorJunctionDetectionPipeline colorJunctionRightPipeline =
            new ColorJunctionDetectionPipeline(false);

    public static ContourStackPipeline contourStackPipeline = new ContourStackPipeline();

    public static MotorEx leftOdo;
    public static MotorEx rightOdo;
    public static MotorEx backOdo;

    public static BNO055IMU imu;

    public static float xLoc = 0;
    public static float yLoc = 0;
    public static float rotation = 0;

    double[] tune = {1.0, 1.0, 1.0, 1.0};

    private static final char squareXLocation = 'A';
    private static final int squareYLocation = 1;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private static LinearOpMode opMode;
    private static boolean opIsAuto;
    public static boolean useFTCDash;
    public static final ElapsedTime period = new ElapsedTime();

    //All distance values in inches
    private static final float wheelRadius = (float) 1.49606 / 2;
    private static final float ticksPerRev = 1440;
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    private static final double TRACKWIDTH = trackTuning;
    private static final double CENTER_WHEEL_OFFSET = -4.9375; //Was 7.75


    //Create the Subsystems
    public static DrivetrainSubsystem drivetrain;
    public static HolonomicOdometry holOdom;

    public static PositionalMovementSubsystem positionalMovement;


    /**
     * Initalize the robot object.
     *
     * @param OPMode  <b>Always</b> pass <i>this</i> here as it allows the telemetry to initialize.
     * @param isAuto  Tells the robot whether or not to add telemetry for autonomous
     * @param useDash Tells the robot whether to send data to the FTC Dashboard
     */
    public Robot(LinearOpMode OPMode, boolean isAuto, boolean useDash) {
        //Grab opmode object from active opMode for telemetry
        opMode = OPMode;
        opIsAuto = isAuto;
        useFTCDash = useDash;
    }

    private void initHelp(HardwareMap ahwMap, cameraInit cam) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        leftFront = new MotorEx(hwMap, "leftFront", Motor.GoBILDA.RPM_312);
//        rightFront = new MotorEx(hwMap, "rightFront", Motor.GoBILDA.RPM_312);
//        leftBack = new MotorEx(hwMap, "leftBack", Motor.GoBILDA.RPM_312);
//        rightBack = new MotorEx(hwMap, "rightBack", Motor.GoBILDA.RPM_312);
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");

        armLift = hwMap.get(DcMotor.class, "armLift");

        pickupLeft = hwMap.get(CRServo.class, "pickupLeft");
        pickupRight = hwMap.get(CRServo.class, "pickupRight");

        coneTouch = hwMap.get(DigitalChannel.class, "coneTouch");
        armTouch = hwMap.get(DigitalChannel.class, "armTouch");

        leftOdo = new MotorEx(hwMap, "leftOdo");
        rightOdo = new MotorEx(hwMap, "rightOdo");
        backOdo = new MotorEx(hwMap, "backOdo");

        //Set the drivemotors to run without encoder
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set them to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        coneTouch.setMode(DigitalChannel.Mode.INPUT);
        armTouch.setMode(DigitalChannel.Mode.INPUT);

        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo.setDistancePerPulse(ticksToIn);
        rightOdo.setDistancePerPulse(ticksToIn);
        backOdo.setDistancePerPulse(ticksToIn);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rightOdo.resetEncoder();
        leftOdo.resetEncoder();
        backOdo.resetEncoder();

        leftOdo.setInverted(false);
        rightOdo.setInverted(true);
        backOdo.setInverted(true);

        //Apparently it likes the odos flipped???
        holOdom = new HolonomicOdometry(
                () -> rightOdo.getCurrentPosition() * ticksToIn,
                () -> leftOdo.getCurrentPosition() * ticksToIn,
                () -> backOdo.getCurrentPosition() * ticksToIn,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        //ButtonToggleSubsystem.clearList();
        if (cam != Robot.cameraInit.NO_CAM) {
            initCV(hwMap, cam);
        }

        LiftSubsystem.init(armLift, rightOdo, armTouch, opMode);

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
    }

    public void init(HardwareMap ahwMap, cameraInit cam) {
        initHelp(ahwMap, cam);

        holOdom.updatePose(new Pose2d());

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();

        drivetrain = new DrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
        positionalMovement = new PositionalMovementSubsystem(drivetrain, holOdom, opMode);
        drivetrain.resetEncoders();
    }

    public void init(HardwareMap ahwMap, double startX, double startY, double startDegrees, cameraInit cam) {
        initHelp(ahwMap, cam);

        Pose2d startPose = new Pose2d(startX, startY, Rotation2d.fromDegrees(startDegrees));

        holOdom.updatePose(startPose);

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();

        drivetrain = new DrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
        positionalMovement = new PositionalMovementSubsystem(drivetrain, holOdom, opMode);
        drivetrain.resetEncoders();
    }

    /**
     * This function is automatically run within init(..., initCam = true). Should only be ran for debugging
     *
     * @param ahwMap
     */
    public static void initCV(HardwareMap ahwMap, cameraInit cam) {
        HardwareMap hwMap = ahwMap;

        cameraInit = cam;

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        if (cam == COLOR_SLEEVE) {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.setPipeline(colorSleevePipeline);
                    camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } else if (cam == APRIL_SLEEVE) {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.setPipeline(apriltagSleevePipeline);
                    camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } else if (cam == TWO_CAM_JUNCTION) {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.setPipeline(colorJunctionLeftPipeline);
                    camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            int cameraMonitorViewId2 = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            WebcamName webcamName2 = hwMap.get(WebcamName.class, "Webcam 2");
            camera2 = OpenCvCameraFactory.getInstance().createWebcam(webcamName2, cameraMonitorViewId2);

            camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera2.setPipeline(colorJunctionRightPipeline);
                    camera2.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } else if (cam == CONE_STACK) {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.setPipeline(contourStackPipeline);
                    camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } else if(cam == ONE_CAM_JUNCTION) {
//            int cameraMonitorViewId2 = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//            WebcamName webcamName2 = hwMap.get(WebcamName.class, "Webcam 2");
//            camera2 = OpenCvCameraFactory.getInstance().createWebcam(webcamName2, cameraMonitorViewId2);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.setPipeline(colorJunctionRightPipeline);
                    camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                    throw new RuntimeException("Camera could not be opened");
                }
            });
            
        }

    }

    public void switchCamPipeline(cameraInit cam) {
        if(cameraInit == TWO_CAM_JUNCTION) {
            camera2.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {

                }
            });
        }

        switch (cam) {
            case COLOR_SLEEVE:
                camera.setPipeline(colorSleevePipeline);
                break;
            case APRIL_SLEEVE:
                camera.setPipeline(apriltagSleevePipeline);
                break;
            case TWO_CAM_JUNCTION:
                camera.setPipeline(colorJunctionLeftPipeline);
                camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.setPipeline(colorJunctionRightPipeline);
                        camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });
                break;
        }
        cameraInit = cam;
    }

    public enum cameraInit {
        NO_CAM, COLOR_SLEEVE, APRIL_SLEEVE, TWO_CAM_JUNCTION, CONE_STACK, ONE_CAM_JUNCTION
    }

}
