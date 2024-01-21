package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMBASE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.BACKMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.LEFTMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.OFFSETMULT;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.RIGHTMULT;
import static java.lang.Math.PI;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
public class NewRobot2023 {
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

    private boolean visionInit = false;
    private boolean startLocationBasedOnApril = false;
    private double recalibrateTime = 0;

    public static final String LOGCATTAG = "Robot Logging: ";

    //Tuning Values
    float colorGain = 2;
    double trackwidth = 14.14;
    double odoOffset = 6.5515;
    public static int base = ARMBASE;

    //Subsystems
    public HoloDrivetrainSubsystem holoDrivetrain;
    public static HolonomicOdometry holOdom;
    public MovementSubsystem movementSubsystem;
    public ArmSubsystem armSubsystem;
    public CameraSubsystem cameraSubsystem;
    public ColorDetectionSubsystem colorDetectionSubsystem;

    /**
     * @param s The minimum time between recalibrating position based on AprilTag.
     * Once the time has elapsed, the robot will reopen the camera stream, and attempt
     * to recalibrate its position. Once it has done so, it will reclose the camera stream.
     */
    public NewRobot2023(double s) {
        recalibrateTime = s;
    }

    public NewRobot2023() {
        recalibrateTime = -1;
    }

    public void init(HardwareMap ahwMap, boolean initVision, LinearOpMode opMode) {
        leftFront = new MotorEx(ahwMap, "leftFront");
        leftBack = new MotorEx(ahwMap, "leftBack");
        rightFront = new MotorEx(ahwMap, "rightFront");
        rightBack = new MotorEx(ahwMap, "rightBack");

        leftFront.setInverted(true);
        rightBack.setInverted(true);

        holoDrivetrain = new HoloDrivetrainSubsystem(leftFront, rightFront, leftBack, rightBack);
    }

    public boolean isVisionInit() {
        return visionInit;
    }
}
