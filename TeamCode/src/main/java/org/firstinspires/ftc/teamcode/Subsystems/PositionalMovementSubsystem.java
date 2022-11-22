package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.leftOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightBack;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightFront;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.rightOdo;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.useFTCDash;
import static org.firstinspires.ftc.teamcode.Subsystems.tuningConstants.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.s;
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
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        } else {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
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
                    drive.l.setPower(rampDownSpeed);
                    leftBack.setPower(rampDownSpeed);
                    rightFront.setPower(-rampDownSpeed);
                    rightBack.setPower(-rampDownSpeed);
                     */
                    drive.drive(rampDownSpeed, -rampDownSpeed);
                } else {
                    /*
                    drive.l.setPower(-rampDownSpeed);
                    leftBack.setPower(-rampDownSpeed);
                    rightFront.setPower(rampDownSpeed);
                    rightBack.setPower(rampDownSpeed);
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
     * Move the robot to a given point from the current location
     * @param endX The X coordinate to go to
     * @param endY The Y cooridnate to go to
     * @param endAngle The end angle of the movement (In Radians)
     * @param speed The maximum speed at which to move the motors (0.0 - 1.0)
     * @param holometric Whether the robot should move holometrically (true) or to turn and drive straight (false)
     * @param dashTelemetry Pass the telemetry object from the opmode to this parameter to send data to the dashboard
     */
    public static void moveTo(double endX, double endY, double endAngle, @FloatRange(from = 0, to = 1) double speed, boolean holometric, Telemetry dashTelemetry) {
        final double distanceTol = 0.5;
        final double turnTol = 3.14 / 6;
        boolean xTrac = true, yTrac = true;

        if (!holometric) {
            moveToLocation(endX, endY, speed);
        }

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        double startX = moving.getX(), startY = moving.getY(), heading = moving.getHeading();

        PIDController pidX = new PIDController(FORWARDPIDFP, FORWARDPIDFI, FORWARDPIDFD);
        PIDController pidY = new PIDController(HORIZONTALPIDFP, HORIZONTALPIDFI, HORIZONTALPIDFD);
        PIDController pidT = new PIDController(0,0,0);

        pidX.setTolerance(distanceTol);
        pidY.setTolerance(distanceTol);
        pidT.setTolerance(turnTol);

        double x = pidX.calculate(startX, endX);
        double y = pidY.calculate(startY, endY);
        double t = 0;

        double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
        double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

        double lfSpeed = x_rotated + y_rotated;
        double lbSpeed = x_rotated - y_rotated;
        double rfSpeed = x_rotated - y_rotated;
        double rbSpeed = x_rotated + y_rotated;

        double lfTemp = abs(lfSpeed);
        double lbTemp = abs(lbSpeed);
        double rfTemp = abs(rfSpeed);
        double rbTemp = abs(rbSpeed);

        double maxSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

        if(maxSpeed > 1) {
            lfSpeed /= maxSpeed;
            lbSpeed /= maxSpeed;
            rfSpeed /= maxSpeed;
            rbSpeed /= maxSpeed;
        }


        double curX, curY, curT;

        //Allow time to setup tracking on FTCDashboard
        while (opMode.opModeIsActive() && !opMode.gamepad1.x && useFTCDash) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();
            curT = moving.getHeading();

            dashTelemetry.addData("X Error", endX - curX);
            dashTelemetry.addData("X PID Output", x);
            dashTelemetry.addData("Y Error", endY - curY);
            dashTelemetry.addData("Y PID Output", y);
            dashTelemetry.addData("Angle", curT);
            dashTelemetry.addData("LF Speed", lfSpeed * speed);
            dashTelemetry.addData("LB Speed", rfSpeed * speed);
            dashTelemetry.addData("RF Speed", rfSpeed * speed);
            dashTelemetry.addData("RB Speed", rbSpeed * speed);
            dashTelemetry.addData("Tracking X", xTrac);
            dashTelemetry.addData("Tracking Y", yTrac);
            dashTelemetry.update();
        }

//        leftFront.setPower(lfSpeed * speed);
//        leftBack.setPower(rfSpeed * speed);
//        rightFront.setPower(rfSpeed * speed);
//        rightBack.setPower(rbSpeed * speed);
//
        drive.drive(lfSpeed * speed, rfSpeed * speed, lbSpeed * speed, rbSpeed * speed);

        while(opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();
            curT = moving.getHeading();

            x = pidX.calculate(curX, endX);
            y = pidY.calculate(curY, endY);

//            if(xTrac) {
//                x = pidX.calculate(curX);
//            } else {
//                x = 0;
//            }
//            if(yTrac) {
//                y = pidY.calculate(curY);
//            } else {
//                y = 0;
//            }

            //t = pidT.calculate(curT);

            x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
            y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

            // x, y, theta input mixing
            lfSpeed = x_rotated + y_rotated;
            lbSpeed = x_rotated - y_rotated;
            rfSpeed = x_rotated - y_rotated;
            rbSpeed = x_rotated + y_rotated;

            lfTemp = abs(lfSpeed);
            lbTemp = abs(lbSpeed);
            rfTemp = abs(rfSpeed);
            rbTemp = abs(rbSpeed);

//            if(lfTemp > 1.0 && lfTemp >= lbTemp && lfTemp >= rfTemp && lfTemp >= rbTemp) {
//                lbSpeed = lbSpeed / lfTemp;
//                rfSpeed = rfSpeed / lfTemp;
//                rbSpeed = rbSpeed / lfTemp;
//                lfSpeed = lfSpeed / lfTemp;
//            } else if (lbTemp > 1.0 && lbTemp >= lfTemp && lbTemp >= rfTemp && lbTemp >= rbTemp) {
//                lfSpeed = lfSpeed / lbTemp;
//                rfSpeed = rfSpeed / lbTemp;
//                rbSpeed = rbSpeed / lbTemp;
//                lbSpeed = lfSpeed / lbTemp;
//            } else if (rfTemp > 1.0 && rfTemp >= lfTemp && rfTemp >= lbTemp && rfTemp >= rbTemp) {
//                lfSpeed = lfSpeed / rfTemp;
//                lbSpeed = lbSpeed / rfTemp;
//                rbSpeed = rbSpeed / rfTemp;
//                rfSpeed = rfSpeed / rfTemp;
//            } else if (rbTemp > 1.0 && rbTemp >= lfTemp && rbTemp >= lbTemp && rbTemp >= rfTemp) {
//                lfSpeed = lfSpeed / rbTemp;
//                lbSpeed = lbSpeed / rbTemp;
//                rfSpeed = rfSpeed / rbTemp;
//                rbSpeed = rbSpeed / rbTemp;
//            }

           maxSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if(maxSpeed > 1) {
                lfSpeed /= maxSpeed;
                lbSpeed /= maxSpeed;
                rfSpeed /= maxSpeed;
                rbSpeed /= maxSpeed;
            }

//            leftFront.setPower(lfSpeed * speed);
//            leftBack.setPower(rfSpeed * speed);
//            rightFront.setPower(rfSpeed * speed);
//            rightBack.setPower(rbSpeed * speed);

            drive.drive(lfSpeed * speed, rfSpeed * speed, lbSpeed * speed, rbSpeed * speed);

            if(pidX.atSetPoint() && pidY.atSetPoint() /* && pidT.atSetPoint()*/) {
                break;
            }


            dashTelemetry.addData("X Error", endX - curX);
            dashTelemetry.addData("X PID Output", x);
            dashTelemetry.addData("Y Error", endY - curY);
            dashTelemetry.addData("Y PID Output", y);
            dashTelemetry.addData("Angle", curT);
            dashTelemetry.addData("LF Speed", lfSpeed * speed);
            dashTelemetry.addData("LB Speed", lfSpeed * speed);
            dashTelemetry.addData("RF Speed", rfSpeed * speed);
            dashTelemetry.addData("RB Speed", rbSpeed * speed);
            dashTelemetry.addData("Tracking X", xTrac);
            dashTelemetry.addData("Tracking Y", yTrac);
            dashTelemetry.update();

        }
        drive.stop();
    }

    public static void moveTo(double endX, double endY, double speed, boolean holometric) {
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        double heading = moving.getHeading();
        moveTo(endX, endY, heading, speed, holometric, null);
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
        //PIDFController pidfTurning =
                //new PIDFController(ANGLEPIDFP, ANGLEPIDFI, ANGLEPIDFD, ANGLEPIDFF, 0, heading - targetAngle);

        if(useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);

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
            leftBack.setPower(adjustedLeftSpeed);
            leftFront.setPower(adjustedLeftSpeed);
            rightBack.setPower(adjustedRightSpeed);
            rightFront.setPower(adjustedRightSpeed);

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

                leftBack.setPower(x_rotated);
                leftFront.setPower(x_rotated);
                rightBack.setPower(x_rotated);
                rightFront.setPower(x_rotated);

                lastCorrect = Robot.period.time();
            }

            */



        }

        //drive.stop();

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

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
//        PIDFController pidfTurning =
//                new PIDFController(ANGLEPIDFP, ANGLEPIDFI, ANGLEPIDFD, ANGLEPIDFF, 0, heading - targetAngle);

        if(useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);

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
            leftBack.setPower(adjustedLeftSpeed);
            leftFront.setPower(adjustedLeftSpeed);
            rightBack.setPower(adjustedRightSpeed);
            rightFront.setPower(adjustedRightSpeed);

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

                leftBack.setPower(x_rotated);
                leftFront.setPower(x_rotated);
                rightBack.setPower(x_rotated);
                rightFront.setPower(x_rotated);

                lastCorrect = Robot.period.time();
            }

            */



        }

        //drive.stop();

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    public static void moveWithLookAhead(double endX, double endY, double speed, double lookAheadDis) {
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        final double moveTolerance = 1;
        PIDController pid = new PIDController(DistancePIDFP, DistancePIDFI, DistancePIDFD);
        final double startX = moving.getX(), startY = moving.getY();
        double x = moving.getX(), y = moving.getY(), heading = moving.getHeading() + PI;
        double deltaX = endX - x, deltaY = endY - y;
        double distance = Math.hypot(deltaX, deltaY);
        double targetAngle = Math.atan2(deltaY, deltaX);
        //Cap the output of the PID controller
        double output = Math.max(pid.calculate(distance, 0), 1);
        double xPower = output * Math.cos(targetAngle);
        double yPower = output * Math.sin(targetAngle);
        double x_rotated = xPower * Math.cos(heading) - yPower * Math.sin(heading);
        double y_rotated = xPower * Math.sin(heading) + yPower * Math.cos(heading);

        double lfSpeed = speed * (x_rotated + y_rotated + heading);
        double rfSpeed = speed * (x_rotated - y_rotated - heading);
        double lbSpeed = speed * (x_rotated - y_rotated + heading);
        double rbSpeed = speed * (x_rotated + y_rotated - heading);

        Telemetry telemetry = null;
        if(useFTCDash) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
        }

        while(opMode.opModeIsActive() && !opMode.gamepad1.x) {
            telemetry.addData("Distance", distance);
            telemetry.addData("PID Output", output);
            telemetry.addData("Left Front", lfSpeed);
            telemetry.addData("Left Back", lbSpeed);
            telemetry.addData("Right Front", rfSpeed);
            telemetry.addData("Right Back", rbSpeed);
            telemetry.addData("Target Angle", targetAngle);

            telemetry.update();
        }

        while (opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            x = moving.getX();
            y = moving.getY();
            heading = moving.getHeading();

            deltaX = endX - x;
            deltaY = endY - y;
            distance = Math.hypot(deltaX, deltaY);
            targetAngle = Math.atan2(deltaY, deltaX);
            output = pid.calculate(distance, 0);
            xPower = output * Math.cos(targetAngle);
            yPower = output * Math.sin(targetAngle);
            x_rotated = xPower * Math.cos(heading) - yPower * Math.sin(heading);
            y_rotated = xPower * Math.sin(heading) + yPower * Math.cos(heading);

            lfSpeed = speed * (x_rotated + y_rotated + heading);
            rfSpeed = speed * (-x_rotated + y_rotated - heading);
            lbSpeed = speed * (-x_rotated + y_rotated + heading);
            rbSpeed = speed * (x_rotated + y_rotated - heading);

            if (distance <= moveTolerance) {
                break;
            }

            leftFront.setPower(lfSpeed);
            rightFront.setPower(rfSpeed);
            leftBack.setPower(lbSpeed);
            rightBack.setPower(rbSpeed);

            if(useFTCDash) {
                telemetry.addData("Distance", distance);
                telemetry.addData("PID Output", output);
                telemetry.addData("Left Front", lfSpeed);
                telemetry.addData("Left Back", lbSpeed);
                telemetry.addData("Right Front", rfSpeed);
                telemetry.addData("Right Back", rbSpeed);

                telemetry.update();
            }
        }
        drive.stop();
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
