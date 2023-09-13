package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import javax.annotation.Nullable;

/**
 * Tested and Working Features:
 * - Holonomic Odometry
 * - Smooth Deceleration
 * <p>
 * Implemented, but not tested/working:
 * - AprilTag Positioning
 *
 * <p>
 * Untested/Unimplemented Features:
 * - Error Storage & Logs
 * - Movement Library
 * - Error correction
 */
public class Robot2023 {
    //All values in IN
    private static final float wheelRadius = (float) 1.37795 / 2;
    private static final float ticksPerRev = 1440;
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    public static HolonomicOdometry holOdom;

    public static MotorEx leftFront;
    public static MotorEx leftBack;
    public static MotorEx rightFront;
    public static MotorEx rightBack;

    public static IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private boolean visionInit = false;
    private boolean startLocationBasedOnApril = false;
    private double recalibrateTime = 0;

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

    public void init(HardwareMap ahwMap, boolean initVision) {
        leftFront = new MotorEx(ahwMap, "leftFront");
        leftBack = new MotorEx(ahwMap, "leftBack");
        rightFront = new MotorEx(ahwMap, "rightFront");
        rightBack = new MotorEx(ahwMap, "rightBack");

        imu = ahwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setInverted(true);
        leftBack.setInverted(true);

        leftBack.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        leftBack.setRunMode(MotorEx.RunMode.RawPower);
        leftFront.setRunMode(MotorEx.RunMode.RawPower);
        rightBack.setRunMode(MotorEx.RunMode.RawPower);
        rightFront.setRunMode(MotorEx.RunMode.RawPower);

        leftFront.setDistancePerPulse(ticksToIn);
        leftBack.setDistancePerPulse(ticksToIn);
        rightFront.setDistancePerPulse(ticksToIn);

        leftFront.resetEncoder();
        leftBack.resetEncoder();
        rightFront.resetEncoder();

        holOdom = new HolonomicOdometry(
                () -> (leftBack.getCurrentPosition() * -ticksToIn),
                () -> leftFront.getCurrentPosition() * ticksToIn,
                () -> rightFront.getCurrentPosition() * ticksToIn,
                10.375,
                -3.8125
        );

        holOdom.updatePose();
        holOdom.updatePose(new Pose2d());

        if(initVision) {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawTagID(true)
                    .setDrawAxes(false)
                    .setDrawTagOutline(true)
                    .setDrawCubeProjection(false)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setTagLibrary(getCurrentGameTagLibrary())
                    .setLensIntrinsics(520.549, 520.549, 313.018, 237.164)
                    .build();


            visionPortal = new VisionPortal.Builder()
                    .addProcessor(aprilTagProcessor)
                    .enableCameraMonitoring(true)
                    .setAutoStopLiveView(true)
                    .setCamera(ahwMap.get(WebcamName.class, "Webcam"))
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .build();


            visionInit = true;
            //Try and set location based on AprilTag. If it fails, set to 0,0,0
            initFindPosition();
        }


    }

    public void initFindPosition() {
        Pose2d pose = getPoseFromAprilTag();
        if(pose != null) {
            holOdom.updatePose(pose);
            startLocationBasedOnApril = true;

        }
    }

    @Nullable
    public Pose2d getPoseFromAprilTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection: detections) {
            if (detection.metadata != null) {
                return translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
            }
        }

        return null;
    }

    private Pose2d translateToCam(double x, double y, double z, double yaw, double pitch, double roll) {
        yaw = Math.toRadians(yaw);
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        RealMatrix yawMatrix = new Array2DRowRealMatrix(new double[][] {
                {cos(yaw), -sin(yaw), 0},
                {sin(yaw), cos(yaw), 0},
                {0, 0, 1}
        });
        RealMatrix pitchMatrix = new Array2DRowRealMatrix(new double[][] {
                {cos(pitch), 0, sin(pitch)},
                {0, 1, 0},
                {-sin(pitch), 0, cos(pitch)}
        });
        RealMatrix rollMatrix = new Array2DRowRealMatrix(new double[][] {
                {1, 0, 0},
                {0, cos(roll), -sin(roll)},
                {0, sin(roll), cos(roll)}
        });
        RealMatrix rotationMatrix = yawMatrix.multiply(pitchMatrix).multiply(rollMatrix);

        RealMatrix tAprilToCamMatrix = new Array2DRowRealMatrix(new double[][] {
                {rotationMatrix.getEntry(0, 0), rotationMatrix.getEntry(0, 1), rotationMatrix.getEntry(0, 2), x},
                {rotationMatrix.getEntry(1, 0), rotationMatrix.getEntry(1, 1), rotationMatrix.getEntry(1, 2), y},
                {rotationMatrix.getEntry(2, 0), rotationMatrix.getEntry(2, 1), rotationMatrix.getEntry(2, 2), z},
                {0, 0, 0, 1}
        });

        RealMatrix tCamToAprilMatrix = MatrixUtils.inverse(tAprilToCamMatrix);

        //What threshold to use? Not sure if it matters
        Rotation rotationExtract = new Rotation(tCamToAprilMatrix.getSubMatrix(0, 2, 0, 2).getData(), 0.1);

        double[] angles = rotationExtract.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR);

        return new Pose2d(
                tCamToAprilMatrix.getEntry(1, 3),
                tCamToAprilMatrix.getEntry(0, 3),
                new Rotation2d(angles[0])
        );
    }

    /**
     * Smooth Deceleration for Evan.
     * Tune to feel, too small max D will feel unresponsive, so much will feel abrupt.
     * Always call in loop to achieve desired power
     * @param motor The motor to set power to
     * @param targetPower The power to set the motor to
     */
    public void smoothDecelerate(MotorEx motor, double targetPower) {
        double maxDecelerate = 0.25;
        double currentPower = motor.get();
        if (abs(targetPower) > abs(currentPower) - abs(maxDecelerate)) {
            motor.set(targetPower);
        } else {
            motor.set(currentPower + (maxDecelerate * (-1 * signum(currentPower))));
        }
    }

    public void smoothDrive(double x, double y, double turn) {
        double leftFrontPower = y + x + turn;
        double leftBackPower = y - x + turn;
        double rightFrontPower = y - x - turn;
        double rightBackPower = y + x - turn;

        double maxPower = Math.max(Math.max(abs(leftFrontPower), abs(leftBackPower)),
                Math.max(abs(rightFrontPower), abs(rightBackPower)));

        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        smoothDecelerate(leftFront, leftFrontPower);
        smoothDecelerate(leftBack, leftBackPower);
        smoothDecelerate(rightFront, rightFrontPower);
        smoothDecelerate(rightBack, rightBackPower);
    }
}
