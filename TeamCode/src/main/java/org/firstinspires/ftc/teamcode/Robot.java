package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry.Item;

import static org.firstinspires.ftc.teamcode.pathType.*;
import static java.lang.Math.*;

/**
 * This class contains all hardware maps and functions for use in opModes. An object should be
 * created, then during init phase {@link #init init} should be called with the opMode hwmap
 */
public class Robot {
    public static DcMotor leftFront;
    public static DcMotor rightFront;
    public static DcMotor leftBack;
    public static DcMotor rightBack;
    public static DcMotor extension;
    public static DcMotor angle;
    public static CRServo leftSuck;
    public static CRServo rightSuck;

    public static float xLoc = 0;
    public static float yLoc = 0;
    public static float rotation = 0;
    public static float pathMaxYdev = 0;
    public static float pathAvgYDev = 0;
    public static float pathTotalYDev = 0;
    public static float pathAvgCount = 0;
    public static float maxYdev = 0;
    public static float avgYDev = 0;
    public static float totalYDev = 0;
    public static float avgCount = 0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private static OpMode opMode;
    private final ElapsedTime period  = new ElapsedTime();
    private final static float moveTolerance = 1.0f; //Tolerance for X and Y values when moving
    private final static float turnTolerance = 1.0f; //Tolerance for degree values when turning

    //All distance values in inches
    private static final float odoWidthToCenter = 5 / 2;
    private static final float odoLengthToCenter = 6 / 2;
    private static final float wheelRadius = 1;
    private static final float ticksPerRev = 1160;
    private static final float constantOfMovement = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    private static float deltaLeftOdo = 0;
    private static float deltaRightOdo = 0;
    private static float deltaFrontOdo = 0;


    /**
     * Initalize the robot object.
     * @param OPMode <b>Always</b> pass <i>this</i> here as it allows the telemetry to initialize.
     * @param isAuto Tells the robot whether or not to add telemetry for autonomous
     */
    public Robot(OpMode OPMode, boolean isAuto){
        //Grab telemetry object from active opMode
        opMode = OPMode;
        opMode.telemetry.setAutoClear(false); //Stuff will need to be manually removed
        opMode.telemetry.addAction(new Runnable() { @Override public void run() {updateLocation();} });
        opMode.telemetry.addData("Robot X", ".3f%", xLoc)
                .addData(" Robot Y", ".3f%", yLoc)
                .addData(" Robot Angle", ".3f%", rotation);
        if(isAuto) {
            opMode.telemetry.addData("Overall yDev Average", ".3f%", pathAvgYDev)
                    .addData("Current yDev Max", ".3f%", pathMaxYdev);
        }
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        extension = hwMap.get(DcMotor.class, "extension");
        angle = hwMap.get(DcMotor.class, "angle");

        leftSuck = hwMap.get(CRServo.class, "leftSuck");
        rightSuck = hwMap.get(CRServo.class, "rightSuck");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void updateLocation() {
        deltaLeftOdo = leftBack.getCurrentPosition() - deltaLeftOdo;
        deltaRightOdo = rightBack.getCurrentPosition() - deltaRightOdo;
        deltaFrontOdo = rightFront.getCurrentPosition() - deltaFrontOdo;


        xLoc += constantOfMovement * ((deltaLeftOdo + deltaRightOdo) / 2);
        yLoc = constantOfMovement *
                (deltaFrontOdo - (odoWidthToCenter * (deltaRightOdo - deltaLeftOdo) / (2 * odoLengthToCenter)));
        rotation += constantOfMovement * ((deltaRightOdo - deltaLeftOdo) / (2 * odoLengthToCenter));
    }

    public static void turnTo(float angle, double speed) {
        if(angle <= rotation + 180) {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        } else {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        }

        while (true) {
            updateLocation();
            opMode.telemetry.update();
            if(rotation >= angle - turnTolerance && rotation <= angle + turnTolerance) {
                break;
            }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     *
     * @param PathType The type of path the robot should take to get to the desired destination
     * @param endX Where the robot should end up on the x plane
     * @param endY Where the robot should end up on the y plane
     * @throws wrongInformationException Throws an error if there is missing or extra information for the pathtype
     */
    public static void moveTo(pathType PathType, float endX, float endY, double speed) {
        //This specific case is built to only deal with straight line movement


        pathMan Path = new pathMan(PathType, endX, endY);

        float movementAngle; //The angle at which the robot moves relative to the line of movement
        float angleToTarget = (float) toDegrees(tan((endY - yLoc) / (endX - xLoc))); //The angle of the direct line to the endpoint

        if(PathType == STRAIGHT) {
            movementAngle = 0;
            turnTo(angleToTarget, speed);
        } else if(PathType == STRAIGHT_NO_TURN) {
            movementAngle = rotation - angleToTarget;
        }

        //TODO: Add NO_TURN math
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        float upperXTolerance = endX + moveTolerance;
        float lowerXTolerance = endX - moveTolerance;
        float upperYTolerance = endY + moveTolerance;
        float lowerYTolerance = endY - moveTolerance;

        pathAvgYDev = 0;
        pathTotalYDev = 0;
        pathAvgCount = 0;

        Object tempUpdateToken = opMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                if(Path.getYDeviation() > pathMaxYdev) pathMaxYdev = Path.getYDeviation();
                pathTotalYDev += Path.getYDeviation();
                pathAvgCount++;
                pathAvgYDev = pathTotalYDev / pathAvgCount;

                if(Path.getYDeviation() > maxYdev) maxYdev = Path.getYDeviation();
                totalYDev += Path.getYDeviation();
                avgCount++;
                avgYDev = totalYDev / avgCount;
            }
        });

        Item movement =
                opMode.telemetry.addData("Current yDev Average", ".3f%", pathAvgYDev)
                        .addData("Current yDev Max", ".3f%", pathMaxYdev);

        while(true) {
            updateLocation();
            opMode.telemetry.update();
            if (xLoc <= upperXTolerance && xLoc >= lowerXTolerance && yLoc <= upperYTolerance && yLoc >= lowerYTolerance) {
                break;
            }
        }

        opMode.telemetry.removeAction(tempUpdateToken);
        opMode.telemetry.removeItem(movement);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        updateLocation();
    }
}
