package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.fieldSquares.getCurrentSquareCenter;
import static org.firstinspires.ftc.teamcode.Subsystems.fieldSquares.getTargetSquareLocation;
import static org.firstinspires.ftc.teamcode.Subsystems.pathType.STRAIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.pathType.STRAIGHT_NO_TURN;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PositionalMovementSubsystem {
    private static DrivetrainSubsystem drive;
    private static HolonomicOdometry holOdom;
    private static OpMode opMode;

    private final static float moveTolerance = 3.0f; //Tolerance for X and Y values when moving
    private final static float turnTolerance = 0.5f; //Tolerance for degree values when turning
    private final static double squareFromCenterTolerance = 6;

    public PositionalMovementSubsystem(DrivetrainSubsystem driveTemp, HolonomicOdometry odoTemp, OpMode opModeTemp) {
        drive = driveTemp;
        holOdom = odoTemp;
        opMode = opModeTemp;
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
            /*
            drive.l.set(-speed);
            leftBack.set(-speed);
            rightFront.set(speed);
            rightBack.set(speed);
            */
            drive.drive(-speed, speed);
            turningRight = false;
        } else {
            /*
            drive.l.set(speed);
            leftBack.set(speed);
            rightFront.set(-speed);
            rightBack.set(-speed);
             */
            drive.drive(speed, -speed);
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
        double angleToTarget = toDegrees(atan2((endY - moving.getY()), (endX - moving.getX()))); //The angle of the direct line to the endpoint
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
                        //drive.driveRobotCentric(0,rampDownSpeed,0);
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
            /*
            if(opMode.gamepad1.a) {
                drive.l.set(0);
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

                        drive.l.set(speed * (yComp + xComp));
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
        drive.stop();

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

        //drive.drive(speed);

        leftBack.set(speed);
        leftFront.set(speed);
        rightBack.set(speed);
        rightFront.set(speed);

        double distance, dHelper, curX, curY;

        while(true) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();

            deltaX = endX - curX;
            deltaY = endY - curY;
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



}
