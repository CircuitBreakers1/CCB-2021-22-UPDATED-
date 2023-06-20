package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.tuningConstants.trackTuning;
import static java.lang.Math.PI;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem;


/**
 * This class contains all hardware maps and functions for use in opModes. An object should be
 * created, then during init phase {@link #init init} should be called with the opMode hwmap.
 * <p>
 * This class also creates a basic framework of telemetry. It sets auto clear to off, meaning that
 * the first time something is intended to be added to telemetry it should be created using
 * .addData() and stored in a Telemetry.Item. To update, use .setValue() with the Item.
 */
public class SummerJohn2023 {
    public static DcMotorEx leftFront;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftBack;
    public static DcMotorEx rightBack;

    public static DcMotorEx leftLift;
    public static DcMotorEx rightLift;
    public static DcMotorEx rotate;

    public static Servo grab;

    public static DigitalChannel rotateTrigger;

    /* local OpMode members. */
    HardwareMap hwMap = null;

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

    public SummerJohn2023() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");

        rotate = hwMap.get(DcMotorEx.class, "rotate");
        leftLift = hwMap.get(DcMotorEx.class, "leftLift");
        rightLift = hwMap.get(DcMotorEx.class, "rightLift");

        grab = hwMap.get(Servo.class, "grab");

        rotateTrigger = hwMap.get(DigitalChannel.class, "rotateTrigger");

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        //Set the motors to reverse direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setArmHeight(int position) {
        leftLift.setTargetPosition(position);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(leftLift.getPower());
    }

    public void maintainArm() {
        rightLift.setPower(leftLift.getPower());
    }
}
