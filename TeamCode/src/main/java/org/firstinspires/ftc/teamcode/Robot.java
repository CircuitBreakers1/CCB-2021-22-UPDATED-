package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.pathType.*;


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
    private final double moveTolerance = 1.0; //Tolerance for X and Y values when moving
    private final double turnTolerance = 1.0; //Tolerance for degree values when turning

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

    }

    /**
     *
     * @param PathType The type of path the robot should take to get to the desired destination
     * @param endX Where the robot should end up on the x plane
     * @param endY Where the robot should end up on the y plane
     * @throws wrongInformationException Throws an error if there is missing or extra information for the pathtype
     */
    public static void moveTo(pathType PathType, float endX, float endY) throws wrongInformationException {
        if(PathType == STRAIGHT_TURN_TO || PathType == ARC_NO_TURN || PathType == ARC_TURN_TO) {
            throw new wrongInformationException();
        }
    }
}
