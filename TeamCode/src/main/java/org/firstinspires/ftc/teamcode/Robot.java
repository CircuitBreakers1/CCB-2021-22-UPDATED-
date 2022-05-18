package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.pathType.*;
import static java.lang.Math.*;


public class Robot {
    public static DcMotor leftFront;
    public static DcMotor rightFront;
    public static DcMotor leftBack;
    public static DcMotor rightBack;

    public static float xLoc;
    public static float yLoc;
    public static float rotation;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private final ElapsedTime period  = new ElapsedTime();
    private static float moveTolerance = 1.0f; //Tolerance for X and Y values when moving
    private static float turnTolerance = 1.0f; //Tolerance for degree values when turning


    /* Constructor */
    public Robot(){}


    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void updateLocation() {
        xLoc = 0;
        yLoc = 0;
        rotation = 0;
    }

    public static void turnTo(float angle) {

    }

    /**
     *
     * @param PathType The type of path the robot should take to get to the desired destination
     * @param endX Where the robot should end up on the x plane
     * @param endY Where the robot should end up on the y plane
     * @throws wrongInformationException Throws an error if there is missing or extra information for the pathtype
     */
    public static void moveTo(pathType PathType, float endX, float endY, double speed) throws wrongInformationException {
        //This specific case is built to only deal with straight line movement
        if(PathType == STRAIGHT_TURN_TO || PathType == ARC_NO_TURN || PathType == ARC_TURN_TO) {
            throw new wrongInformationException();
        }
        float movementAngle; //The angle at which the robot moves relative to the line of movement
        float angleToTarget = (float) toDegrees(tan((endY - yLoc) / (endX - xLoc))); //The angle of the direct line to the endpoint

        if(PathType == STRAIGHT) {
            movementAngle = 0;
            turnTo(angleToTarget);
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

        while(true) {
            updateLocation();
            if (xLoc <= upperXTolerance && xLoc >= lowerXTolerance && yLoc <= upperYTolerance && yLoc >= lowerYTolerance) {
                break;
            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        updateLocation();
    }
}
