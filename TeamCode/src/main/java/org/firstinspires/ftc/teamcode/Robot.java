package org.firstinspires.ftc.teamcode;

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

import static org.firstinspires.ftc.teamcode.fieldSquares.*;
import static org.firstinspires.ftc.teamcode.moveDirection.*;
import static org.firstinspires.ftc.teamcode.pathType.*;
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
    private boolean opIsAuto;
    private final ElapsedTime period  = new ElapsedTime();
    private final static float moveTolerance = 3.0f; //Tolerance for X and Y values when moving
    private final static float turnTolerance = 0.5f; //Tolerance for degree values when turning
    private final static double squareFromCenterTolerance = 6;

    //All distance values in inches
    private static final float wheelRadius = (float) 1.49606 / 2;
    private static final float ticksPerRev = 1440;
    public static final float ticksToIn = (float) ((2 * PI * wheelRadius) / ticksPerRev);

    private static final double TRACKWIDTH = 12;
    private static final double CENTER_WHEEL_OFFSET = 4 + (15/16); //Was 7.75

    public static MecanumDrive drive = null;

    public static HolonomicOdometry holOdom = new HolonomicOdometry(
            () -> leftOdo.getCurrentPosition() * -ticksToIn,
            () -> rightOdo.getCurrentPosition() * ticksToIn,
            () -> backOdo.getCurrentPosition() * -ticksToIn,
            TRACKWIDTH, CENTER_WHEEL_OFFSET
    );


    /**
     * Initalize the robot object.
     * @param OPMode <b>Always</b> pass <i>this</i> here as it allows the telemetry to initialize.
     * @param isAuto Tells the robot whether or not to add telemetry for autonomous
     */
    public Robot(OpMode OPMode, boolean isAuto){
        //Grab telemetry object from active opMode
        opMode = OPMode;
        opIsAuto = isAuto;
    }

    public void init(HardwareMap ahwMap) {
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

        drive = new MecanumDrive(false, leftFront, rightFront, leftBack, rightBack);

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

        holOdom.updatePose(new Pose2d());

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();
    }

    public void init(HardwareMap ahwMap, double startX, double startY, double startDegrees) {
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

        leftBack.setRunMode(Motor.RunMode.VelocityControl);
        leftFront.setRunMode(Motor.RunMode.VelocityControl);
        rightBack.setRunMode(Motor.RunMode.VelocityControl);
        rightFront.setRunMode(Motor.RunMode.VelocityControl);

        coneTouch.setMode(DigitalChannel.Mode.INPUT);

        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /*
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
         */
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBack.setInverted(true);
        leftFront.setInverted(true);

        leftOdo.setDistancePerPulse(ticksToIn);
        rightOdo.setDistancePerPulse(ticksToIn);
        backOdo.setDistancePerPulse(ticksToIn);

        drive = new MecanumDrive(false, leftFront, rightFront, leftBack, rightBack);

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

        Pose2d startPose = new Pose2d(startX, startY, Rotation2d.fromDegrees(startDegrees));

        holOdom.updatePose(startPose);

        Pose2d moving = holOdom.getPose();
        xLoc = (float) moving.getX();
        yLoc = (float) moving.getY();
        rotation = (float) moving.getHeading();
    }

    /**
     * Tells the robot to turn to a specific heading.
     * @param angle The angle to turn to
     * @param speed The speed at which to turn
     * @return The final angle, for debug purposes
     */
    public static double turnTo(double angle, double speed) {
        Pose2d moving = holOdom.getPose();

        double heading = Math.toDegrees(moving.getHeading());
        double tempTargetNormalize, tempCurrentNormalize, tempOppCurrent;

        boolean turningRight;

        tempTargetNormalize = angle < 0 ? angle + 360 : angle;
        tempCurrentNormalize = heading < 0 ? heading + 360 : heading;
        tempOppCurrent = tempCurrentNormalize + 180 < 360 ? tempCurrentNormalize + 180 : tempCurrentNormalize - 180;

        if(tempCurrentNormalize >= tempTargetNormalize - turnTolerance && tempCurrentNormalize <= tempTargetNormalize + turnTolerance) {
            return heading;
        }

        if(tempCurrentNormalize < tempOppCurrent && tempCurrentNormalize > tempTargetNormalize) {
            leftFront.set(-speed);
            leftBack.set(-speed);
            rightFront.set(speed);
            rightBack.set(speed);
            turningRight = false;
        } else {
            leftFront.set(speed);
            leftBack.set(speed);
            rightFront.set(-speed);
            rightBack.set(-speed);
            turningRight = true;
        }

        while (true) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            heading = (float) Math.toDegrees(moving.getHeading());
            tempCurrentNormalize = heading < 0 ? heading + 360 : heading;

            if(tempCurrentNormalize >= tempTargetNormalize - turnTolerance && tempCurrentNormalize <= tempTargetNormalize + turnTolerance) {
                break;
            }

            //Check if remainging movement is under 45 degrees, and if so begin to ramp motors down
            double remainingDegrees = Math.abs(angle - heading);

            if(remainingDegrees < 45) {
                //Degrees for graphing and functions
                double degreesInverted = 45 - remainingDegrees;

                double rampDownSpeed = (-0.5 / 3.14) * Math.atan((0.12 * degreesInverted) - 3.2) + 0.3;

                if(turningRight) {
                    leftFront.set(rampDownSpeed);
                    leftBack.set(rampDownSpeed);
                    rightFront.set(-rampDownSpeed);
                    rightBack.set(-rampDownSpeed);
                } else {
                    leftFront.set(-rampDownSpeed);
                    leftBack.set(-rampDownSpeed);
                    rightFront.set(rampDownSpeed);
                    rightBack.set(rampDownSpeed);
                }
            }

            opMode.telemetry.addData("Robot Angle", heading);
            opMode.telemetry.addData("Robot Angle Normal", tempCurrentNormalize);
            opMode.telemetry.update();
        }
        
        leftFront.set(0);
        leftBack.set(0);
        rightFront.set(0);
        rightBack.set(0);
        
        return heading;
    }

    /**
     *
     * @param PathType The type of path the robot should take to get to the desired destination
     * @param endX Where the robot should end up on the x plane
     * @param endY Where the robot should end up on the y plane
     * @throws wrongInformationException Throws an error if there is missing or extra information for the pathtype
     */
    public static void moveTo(pathType PathType, double endX, double endY, double speed) {
        //This specific case is built to only deal with straight line movement
        final double maxAngleDeviation = Math.toRadians(2);
        //Represents a distance to the target point where the robot gradually reduces speed to increase accuracy
        final double precisionRange = 6;
        boolean isPaused = false;

        Pose2d moving = holOdom.getPose();

        double movementAngle, oppositeAngle, debugTurnToAngle = 0; //The angle at which the robot moves relative to the line of movement
        double adjForwardSpeed, adjSidewaysSpeed; //Adjusted speeds for non-straight movement
        double angleToTarget = toDegrees(atan2((endY - moving.getY()), (endX - moving.getX()))); //The angle of the direct line to the endpoint
        double startAngleToTarget = angleToTarget;
        if(PathType == STRAIGHT) {
            debugTurnToAngle = turnTo(angleToTarget, speed);
            leftFront.set(speed);
            leftBack.set(speed);
            rightFront.set(speed);
            rightBack.set(speed);
        } else if(PathType == STRAIGHT_NO_TURN) {
            movementAngle = moving.getHeading() + angleToTarget;

            double xComp = cos(movementAngle + 3.14);
            double yComp = sin(movementAngle + 3.14);

            leftFront.set(speed * (yComp + xComp));
            leftBack.set(speed * (yComp - xComp));
            rightFront.set(speed * (yComp - xComp));
            rightBack.set(speed * (yComp + xComp));
        }

        Translation2d target = new Translation2d(endX, endY);
        Translation2d pose;

        while(true) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            pose = moving.getTranslation();
            double x = moving.getX(), y = moving.getY(), heading = moving.getHeading();
            double distance = Math.sqrt(((x - endX) * (x - endX)) + ((y - endY) * (y - endY)));

            opMode.telemetry.addData("Robot X", x);
            opMode.telemetry.addData("Robot Y", y);
            opMode.telemetry.addData("Distance", distance);
            if (PathType == STRAIGHT) {
                opMode.telemetry.addData("Robot turnTo() Angle", debugTurnToAngle);
            }
            opMode.telemetry.addData("Robot Angle", Math.toDegrees(moving.getHeading()));
            opMode.telemetry.addData("Robot Target Angle", angleToTarget);
            opMode.telemetry.update();

            if (distance < moveTolerance) {
                break;
            } else if (distance < precisionRange && !isPaused) {
                double distanceInverted = 5 - distance;

                double rampDownSpeed = (-0.6 / 3.14) * Math.atan((1.00 * distanceInverted) - 0.6) + 0.3;

                switch (PathType) {
                    case STRAIGHT:
                        drive.driveRobotCentric(0,rampDownSpeed,0);
                        break;
                    case STRAIGHT_NO_TURN:
                        movementAngle = moving.getHeading() + angleToTarget;

                        double xComp = cos(movementAngle + 3.14);
                        double yComp = sin(movementAngle + 3.14);

                        leftFront.set(rampDownSpeed * (yComp + xComp));
                        leftBack.set(rampDownSpeed * (yComp - xComp));
                        rightFront.set(rampDownSpeed * (yComp - xComp));
                        rightBack.set(rampDownSpeed * (yComp + xComp));
                        break;
                }
            }
            /*
            if(opMode.gamepad1.a) {
                leftFront.set(0);
                leftBack.set(0);
                rightFront.set(0);
                rightBack.set(0);
                isPaused = true;
            }

             */
            /*
            if(opMode.gamepad1.b) {
                switch (PathType) {
                    case STRAIGHT:
                        drive.driveRobotCentric(0,speed,0);
                        break;
                    case STRAIGHT_NO_TURN:
                        movementAngle = moving.getHeading() + angleToTarget;

                        double xComp = cos(movementAngle + 3.14);
                        double yComp = sin(movementAngle + 3.14);

                        leftFront.set(speed * (yComp + xComp));
                        leftBack.set(speed * (yComp - xComp));
                        rightFront.set(speed * (yComp - xComp));
                        rightBack.set(speed * (yComp + xComp));
                        break;
                }
                isPaused = false;
            }

             */


            //Angle Adjustment using recursion
            angleToTarget = toDegrees(atan2((endY - moving.getY()), (endX - moving.getX())));
            /* if(angleToTarget > 360) {
                angleToTarget = angleToTarget - 360;
            }
            */
            /*
            if((heading > angleToTarget + maxAngleDeviation || heading < angleToTarget - maxAngleDeviation) && !isPaused && PathType == STRAIGHT) {
                moveTo(STRAIGHT_NO_TURN, endX, endY, speed);
                break;
            }
            if(PathType == STRAIGHT_NO_TURN &&
                    !((angleToTarget <= startAngleToTarget + maxAngleDeviation) && (angleToTarget >= startAngleToTarget - maxAngleDeviation))) {
                moveTo(STRAIGHT_NO_TURN, endX, endY, speed);
                break;
            }
            */

        }
        leftFront.set(0);
        leftBack.set(0);
        rightFront.set(0);
        rightBack.set(0);

        holOdom.updatePose();
    }


    public static void moveToLocation(double endX, double endY, double speed) {
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();
        final double tolerance = 3;
        double startX = moving.getX(), startY = moving.getY(), heading = moving.getHeading();
        double deltaX = startX - endX, deltaY = startY - endY;
        double targetAngle = atan(deltaY / deltaX);
        targetAngle = targetAngle + 3.14;

        if(targetAngle > 6.28) {
            targetAngle = targetAngle - 6.28;
        }

        turnTo(toDegrees(targetAngle), speed);

        leftFront.set(speed);
        rightFront.set(speed);
        leftBack.set(speed);
        rightBack.set(speed);

        double distance, dHelper, curX, curY;

        while(true) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();

            deltaX = curX - endX;
            deltaY = curY - endY;
            dHelper = (deltaX * deltaX) + (deltaY * deltaY);
            distance = sqrt(dHelper);

            if(distance <= tolerance) {
                break;
            }

            opMode.telemetry.addData("Distance", distance);
            opMode.telemetry.addData("Curr X", curX);
            opMode.telemetry.addData("Curr Y", curY);
            opMode.telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            opMode.telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            opMode.telemetry.update();
        }

        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);

    }

    public static void moveOneSquare(moveDirection direction, double speed) {
        double acceptableDeviationFromCenter = 6;

        Pose2d moving = new Pose2d();
        holOdom.updatePose();
        moving = holOdom.getPose();

        double currX = moving.getX(), currY = moving.getY();


        int[] XYTarget = getTargetSquareLocation(direction, currX, currY);
        int[] XYCurrentIdeal = getCurrentSquareCenter(currX,currY);
        switch (direction) {
            case UP:
            case DOWN:
                if(currX > XYCurrentIdeal[0] + acceptableDeviationFromCenter) {
                    moveTo(STRAIGHT_NO_TURN, XYCurrentIdeal[0] + acceptableDeviationFromCenter, currY, speed);
                } else if (currX < XYCurrentIdeal[0] - acceptableDeviationFromCenter) {
                    moveTo(STRAIGHT_NO_TURN, XYCurrentIdeal[0] - acceptableDeviationFromCenter, currY, speed);
                }
                break;
            case LEFT:
            case RIGHT:
                if(currY > XYCurrentIdeal[1] + acceptableDeviationFromCenter) {
                    moveTo(STRAIGHT_NO_TURN, currX, XYCurrentIdeal[1] + acceptableDeviationFromCenter, speed);
                } else if (currX < XYCurrentIdeal[0] - acceptableDeviationFromCenter) {
                    moveTo(STRAIGHT_NO_TURN, currX, XYCurrentIdeal[0] - acceptableDeviationFromCenter, speed);
                }
                break;
        }

        moveTo(STRAIGHT_NO_TURN, XYTarget[0], XYTarget[1], speed);
    }

}
