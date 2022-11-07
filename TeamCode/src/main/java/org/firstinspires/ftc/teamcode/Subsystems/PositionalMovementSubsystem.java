package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.useFTCDash;
import static org.firstinspires.ftc.teamcode.Subsystems.fieldSquares.getCurrentSquareCenter;
import static org.firstinspires.ftc.teamcode.Subsystems.fieldSquares.getTargetSquareLocation;
import static org.firstinspires.ftc.teamcode.Subsystems.pathType.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.pathType.STRAIGHT_NO_TURN;
import static org.firstinspires.ftc.teamcode.Subsystems.tuningConstants.*;
import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionalMovementSubsystem {
    private static DrivetrainSubsystem drive;
    private static HolonomicOdometry holOdom;
    private static LinearOpMode opMode;

    private final static float moveTolerance = 3.0f; //Tolerance for X and Y values when moving
    private final static float turnTolerance = (float) (PI / 6); //Tolerance for degree values when turning
    private final static double squareFromCenterTolerance = 6;

    public PositionalMovementSubsystem(DrivetrainSubsystem driveTemp, HolonomicOdometry odoTemp, LinearOpMode opModeTemp) {
        drive = driveTemp;
        holOdom = odoTemp;
        opMode = opModeTemp;
    }

    /**
     * Tells the robot to turn to a specific heading.
     * @param angle The angle to turn to IN RADIANS MOTHERFUCKER! Domain is [-pi, pi]
     * @param speed The speed at which to turn
     * @return The final angle, for debug purposes
     */
    public static double turnTo(double angle, double speed) {
        if (!opMode.opModeIsActive()) {
            return 0.0;
        }

        Pose2d moving = holOdom.getPose();

        double heading = moving.getHeading();
        double tempTargetNormalize, tempCurrentNormalize, tempOppCurrent;

        boolean turningRight;

        tempTargetNormalize = angle < 0 ? angle + (2 * PI) : angle;
        tempCurrentNormalize = heading < 0 ? heading + (2 * PI): heading;
        tempOppCurrent = tempCurrentNormalize + PI < (2 * PI) ? tempCurrentNormalize + PI : tempCurrentNormalize - PI;

        double tolCheckCurrent = (2 * PI) + tempCurrentNormalize;
        double tolCheckTarg = (2 * PI) + tempTargetNormalize;

        if(tolCheckCurrent >= tolCheckTarg - turnTolerance && tolCheckCurrent <= tolCheckTarg + turnTolerance) {
            return heading;
        }

        if(tempTargetNormalize >= PI) {
            if(heading >= tempTargetNormalize || heading <= tempOppCurrent) {
                turningRight = true;
            } else {
                turningRight = false;
            }
        } else if (heading <= tempTargetNormalize || heading > tempOppCurrent) {
            turningRight = false;
        } else {
            turningRight = true;
        }

        if(turningRight) {
            leftFront.set(speed);
            leftBack.set(speed);
            rightFront.set(-speed);
            rightBack.set(-speed);
        } else {
            leftFront.set(-speed);
            leftBack.set(-speed);
            rightFront.set(speed);
            rightBack.set(speed);
        }

        while (opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            heading = moving.getHeading();
            tempCurrentNormalize = heading < 0 ? heading + (2 * PI): heading;

            tolCheckCurrent = (2 * PI) + tempCurrentNormalize;

            if(tolCheckCurrent >= tolCheckTarg - turnTolerance && tolCheckCurrent <= tolCheckTarg + turnTolerance) {
                break;
            }

            //Check if remaining movement is under pi/4 radians, and if so begin to ramp motors down
            double remainingDegrees = Math.abs(angle - heading);

            if(remainingDegrees < (PI / 4)) {
                //Degrees for graphing and functions because scuffed PID
                remainingDegrees = Math.toDegrees(remainingDegrees);
                double degreesInverted = 45 - remainingDegrees;

                double rampDownSpeed = (-0.5 / 3.14) * Math.atan((0.12 * degreesInverted) - 3.2) + 0.3;

                if(turningRight) {
                    /*
                    drive.l.set(rampDownSpeed);
                    leftBack.set(rampDownSpeed);
                    rightFront.set(-rampDownSpeed);
                    rightBack.set(-rampDownSpeed);
                     */
                    drive.drive(rampDownSpeed, -rampDownSpeed);
                } else {
                    /*
                    drive.l.set(-rampDownSpeed);
                    leftBack.set(-rampDownSpeed);
                    rightFront.set(rampDownSpeed);
                    rightBack.set(rampDownSpeed);
                     */
                    drive.drive(-rampDownSpeed, rampDownSpeed);
                }
            }
            opMode.telemetry.addData("TURNING", "TURNING");
            opMode.telemetry.addData("Robot Angle", heading);
            opMode.telemetry.addData("Robot Angle Normal", tempCurrentNormalize);
            opMode.telemetry.update();
        }

        drive.stop();

        return heading;
    }

    /**
     *
     * @param PathType The type of path the robot should take to get to the desired destination
     * @param endX Where the robot should end up on the x plane
     * @param endY Where the robot should end up on the y plane
     */
    public static void moveTo(pathType PathType, double endX, double endY, double speed) {
        //This specific case is built to only deal with straight line movement
        final double maxAngleDeviation = Math.toRadians(2);
        //Represents a distance to the target point where the robot gradually reduces speed to increase accuracy
        final double precisionRange = 6;
        boolean isPaused = false;

        Pose2d moving = holOdom.getPose();

        double movementAngle, debugTurnToAngle = 0; //The angle at which the robot moves relative to the line of movement
        double angleToTarget = atan2((endY - moving.getY()), (endX - moving.getX())); //The angle of the direct line to the endpoint
        double startAngleToTarget = angleToTarget;
        if(PathType == STRAIGHT) {
            debugTurnToAngle = turnTo(angleToTarget, speed);
            drive.drive(speed);
        } else if(PathType == STRAIGHT_NO_TURN) {
            movementAngle = moving.getHeading() + angleToTarget;

            double xComp = cos(movementAngle + 3.14);
            double yComp = sin(movementAngle + 3.14);
            /*
            drive.l.set(speed * (yComp + xComp));
            leftBack.set(speed * (yComp - xComp));
            rightFront.set(speed * (yComp - xComp));
            rightBack.set(speed * (yComp + xComp));
             */
            drive.driveHolo(yComp, xComp, 0, speed);
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
            opMode.telemetry.addData("Robot Angle", moving.getHeading());
            opMode.telemetry.addData("Robot Target Angle", angleToTarget);
            opMode.telemetry.update();

            if (distance < moveTolerance) {
                break;
            } else if (distance < precisionRange) {
                double distanceInverted = 5 - distance;

                double rampDownSpeed = (-0.6 / 3.14) * Math.atan((distanceInverted) - 0.6) + 0.3;

                switch (PathType) {
                    case STRAIGHT:
                        drive.drive(rampDownSpeed);
                        break;
                    case STRAIGHT_NO_TURN:
                        movementAngle = moving.getHeading() + angleToTarget;

                        double xComp = cos(movementAngle + 3.14);
                        double yComp = sin(movementAngle + 3.14);

                        /*
                        drive.l.set(rampDownSpeed * (yComp + xComp));
                        leftBack.set(rampDownSpeed * (yComp - xComp));
                        rightFront.set(rampDownSpeed * (yComp - xComp));
                        rightBack.set(rampDownSpeed * (yComp + xComp));
                        */

                        drive.driveHolo(yComp, xComp, 0, rampDownSpeed);
                        break;
                }
            }


            //Angle Adjustment using recursion
            angleToTarget = atan2((endY - moving.getY()), (endX - moving.getX()));
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
        drive.stop();

        holOdom.updatePose();
    }

    public static void moveToLocation(double endX, double endY, double speed) {
        if(!opMode.opModeIsActive()) {
            return;
        }

        Telemetry telemetry = null;
        if(useFTCDash) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
        }

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        final double moveTolerance = 8, angleTolerance = Math.toRadians(2);
        double startX = moving.getX(), startY = moving.getY(), heading = moving.getHeading();
        double deltaX = startX - endX, deltaY = startY - endY;
        double targetAngle = atan2(deltaY, deltaX);
        boolean correctionTriggered = false;
        double lastCorrect;
        targetAngle = targetAngle + 3.14;

        if(targetAngle > 6.28) {
            targetAngle = targetAngle - 6.28;
        }

        heading = turnTo(targetAngle, speed);

        double distance, dHelper, curX, curY;
        double turningAdjustment = 0, speedAdjustment = 0, adjustedLeftSpeed = 0, adjustedRightSpeed = 0;

        dHelper = (deltaX * deltaX) + (deltaY * deltaY);
        distance = sqrt(dHelper);

        //The input of this system is distance to target, the target is 0 distance to the target point, and the output is motor speeds
        PIDFController pidfForward =
                new PIDFController(FORWARDPIDFF, FORWARDPIDFI, FORWARDPIDFD, FORWARDPIDFI, 0, distance);
        //Input: Angle deviation to target angle, Target: 0 angle deviation, Output: Motor weighting
        PIDFController pidfTurning =
                new PIDFController(ANGLEPIDFP, ANGLEPIDFI, ANGLEPIDFD, ANGLEPIDFF, 0, heading - targetAngle);

        if(useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        leftBack.set(speed);
        leftFront.set(speed);
        rightBack.set(speed);
        rightFront.set(speed);

        while(opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            heading = moving.getHeading();

            curX = moving.getX();
            curY = moving.getY();

            deltaX = endX - curX;
            deltaY = endY - curY;
            dHelper = (deltaX * deltaX) + (deltaY * deltaY);
            distance = sqrt(dHelper);

            if(distance <= moveTolerance) {
                break;
            }

            targetAngle = atan(deltaY / deltaX);
            /*

            if(!pidfForward.atSetPoint()) {
                speedAdjustment = pidfForward.calculate(distance);
                adjustedLeftSpeed = speedAdjustment * speed;
                adjustedRightSpeed = speedAdjustment * speed;

                adjustedRightSpeed = clip(adjustedRightSpeed, -speed, speed);
                adjustedRightSpeed = clip(adjustedLeftSpeed, -speed, speed);
            } else {
                adjustedLeftSpeed = speed;
                adjustedRightSpeed = speed;
            }

            /*

            if(!pidfTurning.atSetPoint()) {
                turningAdjustment = pidfTurning.calculate(heading - targetAngle);
                adjustedLeftSpeed = adjustedLeftSpeed - turningAdjustment;
                adjustedRightSpeed = adjustedRightSpeed + turningAdjustment;
            }

             */

            adjustedLeftSpeed = speed - 0.025;
            adjustedRightSpeed = speed;

            //Allow the velocity control to run
            leftBack.set(adjustedLeftSpeed);
            leftFront.set(adjustedLeftSpeed);
            rightBack.set(adjustedRightSpeed);
            rightFront.set(adjustedRightSpeed);

            opMode.telemetry.addData("Distance", distance);
            opMode.telemetry.addData("Curr X", curX);
            opMode.telemetry.addData("Curr Y", curY);
            opMode.telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            opMode.telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            opMode.telemetry.update();

            if(useFTCDash) {
                telemetry.addData("Distance (Forward Error)", distance);
                telemetry.addData("Forward PID Output", speedAdjustment);
                telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
                telemetry.addData("Angle PID Output", turningAdjustment);
                telemetry.addData("Left Speed Target", adjustedLeftSpeed);
                telemetry.addData("Right Speed Target", adjustedRightSpeed);

                telemetry.addData("Curr X", curX);
                telemetry.addData("Curr Y", curY);

                telemetry.update();
            }

            /*
            if((heading > targetAngle + angleTolerance || heading < targetAngle - angleTolerance) && !correctionTriggered) {
                //Trigger correction
                double correctionAngle = atan(deltaY / deltaX);
                correctionTriggered = true;

                double x_rotated = speed * Math.cos(correctionAngle) /* - y * Math.sin(correctionAngle) */ /*;
                double y_rotated = speed * Math.sin(correctionAngle) /* + y * Math.cos(correctionAngle) */ /*;

                leftBack.set(x_rotated);
                leftFront.set(x_rotated);
                rightBack.set(x_rotated);
                rightFront.set(x_rotated);

                lastCorrect = Robot.period.time();
            }

            */



        }

        //drive.stop();

        leftBack.set(0);
        leftFront.set(0);
        rightBack.set(0);
        rightFront.set(0);

    }



    public static void moveToLocation(double endX, double endY, double speed, boolean turn) {
        if(!opMode.opModeIsActive()) {
            return;
        }

        Telemetry telemetry = null;
        if(useFTCDash) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
        }

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        final double moveTolerance = 8, angleTolerance = Math.toRadians(2);
        double startX = moving.getX(), startY = moving.getY(), heading = moving.getHeading();
        double deltaX = startX - endX, deltaY = startY - endY;
        double targetAngle = atan2(deltaY, deltaX);
        boolean correctionTriggered = false;
        double lastCorrect;
        targetAngle = targetAngle + 3.14;

        if(targetAngle > 6.28) {
            targetAngle = targetAngle - 6.28;
        }


        if(turn) {
            heading = turnTo(targetAngle, speed);
        } else {
            heading = 0.0;
        }


        double distance, dHelper, curX, curY;
        double turningAdjustment = 0, speedAdjustment = 0, adjustedLeftSpeed = 0, adjustedRightSpeed = 0;

        dHelper = (deltaX * deltaX) + (deltaY * deltaY);
        distance = sqrt(dHelper);

        //The input of this system is distance to target, the target is 0 distance to the target point, and the output is motor speeds
        PIDFController pidfForward =
                new PIDFController(FORWARDPIDFF, FORWARDPIDFI, FORWARDPIDFD, FORWARDPIDFI, 0, distance);
        //Input: Angle deviation to target angle, Target: 0 angle deviation, Output: Motor weighting
        PIDFController pidfTurning =
                new PIDFController(ANGLEPIDFP, ANGLEPIDFI, ANGLEPIDFD, ANGLEPIDFF, 0, heading - targetAngle);

        if(useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        leftBack.set(speed);
        leftFront.set(speed);
        rightBack.set(speed);
        rightFront.set(speed);

        while(opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            heading = moving.getHeading();

            curX = moving.getX();
            curY = moving.getY();

            deltaX = endX - curX;
            deltaY = endY - curY;
            dHelper = (deltaX * deltaX) + (deltaY * deltaY);
            distance = sqrt(dHelper);

            if(distance <= moveTolerance) {
                break;
            }

            targetAngle = atan(deltaY / deltaX);
            /*

            if(!pidfForward.atSetPoint()) {
                speedAdjustment = pidfForward.calculate(distance);
                adjustedLeftSpeed = speedAdjustment * speed;
                adjustedRightSpeed = speedAdjustment * speed;

                adjustedRightSpeed = clip(adjustedRightSpeed, -speed, speed);
                adjustedRightSpeed = clip(adjustedLeftSpeed, -speed, speed);
            } else {
                adjustedLeftSpeed = speed;
                adjustedRightSpeed = speed;
            }

            /*

            if(!pidfTurning.atSetPoint()) {
                turningAdjustment = pidfTurning.calculate(heading - targetAngle);
                adjustedLeftSpeed = adjustedLeftSpeed - turningAdjustment;
                adjustedRightSpeed = adjustedRightSpeed + turningAdjustment;
            }

             */

            adjustedLeftSpeed = speed - 0.025;
            adjustedRightSpeed = speed;

            //Allow the velocity control to run
            leftBack.set(adjustedLeftSpeed);
            leftFront.set(adjustedLeftSpeed);
            rightBack.set(adjustedRightSpeed);
            rightFront.set(adjustedRightSpeed);

            opMode.telemetry.addData("Distance", distance);
            opMode.telemetry.addData("Curr X", curX);
            opMode.telemetry.addData("Curr Y", curY);
            opMode.telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
            opMode.telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
            opMode.telemetry.update();

            if(useFTCDash) {
                telemetry.addData("Distance (Forward Error)", distance);
                telemetry.addData("Forward PID Output", speedAdjustment);
                telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
                telemetry.addData("Angle PID Output", turningAdjustment);
                telemetry.addData("Left Speed Target", adjustedLeftSpeed);
                telemetry.addData("Right Speed Target", adjustedRightSpeed);

                telemetry.addData("Curr X", curX);
                telemetry.addData("Curr Y", curY);

                telemetry.update();
            }

            /*
            if((heading > targetAngle + angleTolerance || heading < targetAngle - angleTolerance) && !correctionTriggered) {
                //Trigger correction
                double correctionAngle = atan(deltaY / deltaX);
                correctionTriggered = true;

                double x_rotated = speed * Math.cos(correctionAngle) /* - y * Math.sin(correctionAngle) */ /*;
                double y_rotated = speed * Math.sin(correctionAngle) /* + y * Math.cos(correctionAngle) */ /*;

                leftBack.set(x_rotated);
                leftFront.set(x_rotated);
                rightBack.set(x_rotated);
                rightFront.set(x_rotated);

                lastCorrect = Robot.period.time();
            }

            */



        }

        //drive.stop();

        leftBack.set(0);
        leftFront.set(0);
        rightBack.set(0);
        rightFront.set(0);

    }

    public static void moveOneSquare(moveDirection direction, double speed) {
        Pose2d moving = new Pose2d();
        holOdom.updatePose();
        moving = holOdom.getPose();

        double currX = moving.getX(), currY = moving.getY();


        int[] XYTarget = getTargetSquareLocation(direction, currX, currY);
        int[] XYCurrentIdeal = getCurrentSquareCenter(currX,currY);
        switch (direction) {
            case UP:
            case DOWN:
                if(currX > XYCurrentIdeal[0] + squareFromCenterTolerance) {
                    moveTo(STRAIGHT_NO_TURN, XYCurrentIdeal[0] + squareFromCenterTolerance, currY, speed);
                } else if (currX < XYCurrentIdeal[0] - squareFromCenterTolerance) {
                    moveTo(STRAIGHT_NO_TURN, XYCurrentIdeal[0] - squareFromCenterTolerance, currY, speed);
                }
                break;
            case LEFT:
            case RIGHT:
                if(currY > XYCurrentIdeal[1] + squareFromCenterTolerance) {
                    moveTo(STRAIGHT_NO_TURN, currX, XYCurrentIdeal[1] + squareFromCenterTolerance, speed);
                } else if (currX < XYCurrentIdeal[0] - squareFromCenterTolerance) {
                    moveTo(STRAIGHT_NO_TURN, currX, XYCurrentIdeal[0] - squareFromCenterTolerance, speed);
                }
                break;
        }

        moveTo(STRAIGHT_NO_TURN, XYTarget[0], XYTarget[1], speed);
    }


    public static double clip(double input, double bottom, double top) {
        if(input > top) {
            return top;
        } else if(input < bottom) {
            return bottom;
        }

        return input;
    }
}
