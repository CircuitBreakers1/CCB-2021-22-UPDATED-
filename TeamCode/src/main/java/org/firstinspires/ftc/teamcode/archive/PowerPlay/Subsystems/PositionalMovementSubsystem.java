package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.tuningConstants.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.FloatRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionalMovementSubsystem {
    private static DrivetrainSubsystem drive;
    private static HolonomicOdometry holOdom;
    private static LinearOpMode opMode;
    public static Telemetry dashTelemetry = null;

    private final static float moveTolerance = 3.0f; //Tolerance for X and Y values when moving
    private final static float turnTolerance = (float) toRadians(3); //Tolerance for degree values when turning
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
    @Deprecated
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

        if(dashTelemetry != null) {
            while (opMode.opModeIsActive() && !opMode.gamepad1.a){
                dashTelemetry.addData("Error", toDegrees(tolCheckTarg - tolCheckCurrent));
                dashTelemetry.addData("Current Angle", toDegrees(heading));
                dashTelemetry.update();
            }
        }

        if(turningRight) {
            Robot.leftFront.setPower(speed);
            Robot.leftBack.setPower(speed);
            Robot.rightFront.setPower(-speed);
            Robot.rightBack.setPower(-speed);
        } else {
            Robot.leftFront.setPower(-speed);
            Robot.leftBack.setPower(-speed);
            Robot.rightFront.setPower(speed);
            Robot.rightBack.setPower(speed);
        }




        while (opMode.opModeIsActive()) {
            LiftSubsystem.updatePositional();
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

                double p = 0.01;
                double rampDownSpeed = Math.max(abs(p * remainingDegrees), 0.2);

                if(turningRight) {
                    drive.drive(rampDownSpeed, -rampDownSpeed);
                } else {
                    drive.drive(-rampDownSpeed, rampDownSpeed);
                }
            }
            opMode.telemetry.addData("TURNING", "TURNING");
            opMode.telemetry.addData("Robot Angle", heading);
            opMode.telemetry.addData("Robot Angle Normal", tempCurrentNormalize);
            opMode.telemetry.update();

            if(dashTelemetry != null) {
                dashTelemetry.addData("Error", toDegrees(tolCheckTarg - tolCheckCurrent));
                dashTelemetry.addData("Current Angle", toDegrees(heading));
                dashTelemetry.update();
            }

        }

        drive.stop();

        return heading;
    }

    /**
     * Move the robot to a given point from the current location
     * @param endX The X coordinate to go to
     * @param endY The Y cooridnate to go to
     * @param endAngle The end angle of the movement (In Degrees)
     * @param speed The maximum speed at which to move the motors (0.0 - 1.0)
     * @param holometric Whether the robot should move holometrically (true) or to turn and drive straight (false)
     * @param dashTelemetry Pass the telemetry object from the opmode to this parameter to send data to the dashboard
     */
    public static void moveTo(double endX, double endY, double endAngle, @FloatRange(from = 0, to = 1) double speed, boolean holometric, boolean exact, Telemetry dashTelemetry) {
        final double distanceTol = exact ? 0.75 : 2;
        final double turnTol = toRadians(5);
        final double radTarget = toRadians(endAngle);

        if (!holometric) {
            moveToLocation(endX, endY, speed);
        }

        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        double startX = moving.getX(), startY = moving.getY(), heading = -moving.getHeading();

        PIDController pidX = new PIDController(FORWARDPIDFP, FORWARDPIDFI, FORWARDPIDFD);
        PIDController pidY = new PIDController(FORWARDPIDFP, FORWARDPIDFI, FORWARDPIDFD);
        PIDController pidT = new PIDController(ANGLEPIDFP, ANGLEPIDFI, ANGLEPIDFD);

        pidX.setTolerance(distanceTol);
        pidY.setTolerance(distanceTol);
        //pidT.setTolerance(turnTol);

        double x = pidX.calculate(startX, endX);
        double y = pidY.calculate(startY, endY);
        //double t = pidT.calculate(-heading, radTarget);

        double x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
        double y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

        double lfSpeed = x_rotated - y_rotated;
        double lbSpeed = x_rotated + y_rotated;
        double rfSpeed = x_rotated + y_rotated;
        double rbSpeed = x_rotated - y_rotated;

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

//        lfSpeed -= t;
//        lbSpeed -= t;
//        rfSpeed += t;
//        rbSpeed += t;
//
//        lfTemp = abs(lfSpeed);
//        lbTemp = abs(lbSpeed);
//        rfTemp = abs(rfSpeed);
//        rbTemp = abs(rbSpeed);
//
//        maxSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));
//
//        if(maxSpeed > 1) {
//            lfSpeed /= maxSpeed;
//            lbSpeed /= maxSpeed;
//            rfSpeed /= maxSpeed;
//            rbSpeed /= maxSpeed;
//        }

        double curX, curY, curT;

        //Allow time to setup tracking on FTCDashboard
        while (opMode.opModeIsActive() && !opMode.gamepad1.x && Robot.useFTCDash && dashTelemetry != null) {
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();
            curT = moving.getHeading();

            dashTelemetry.addData("X Error", endX - curX);
            dashTelemetry.addData("X PID Output", x);
            dashTelemetry.addData("Y Error", endY - curY);
            dashTelemetry.addData("Y PID Output", y);
            dashTelemetry.addData("T Error", pidT.getPositionError());
            //dashTelemetry.addData("T PID Output", t);
            dashTelemetry.addData("Angle", curT);
            dashTelemetry.addData("LF Speed", lfSpeed * speed);
            dashTelemetry.addData("LB Speed", rfSpeed * speed);
            dashTelemetry.addData("RF Speed", rfSpeed * speed);
            dashTelemetry.addData("RB Speed", rbSpeed * speed);
            dashTelemetry.addData("Current X", curX);
            dashTelemetry.addData("Current Y", curY);
            dashTelemetry.update();
        }

        drive.drive(lfSpeed * speed, rfSpeed * speed, lbSpeed * speed, rbSpeed * speed);

        double cycleTime;
        double lastCycle;

        while(opMode.opModeIsActive()) {
            lastCycle = opMode.getRuntime();
            holOdom.updatePose();
            moving = holOdom.getPose();

            curX = moving.getX();
            curY = moving.getY();
            curT = -moving.getHeading(); //Inverse heading for rotation equation

            x = pidX.calculate(curX);
            y = pidY.calculate(curY);
//            t = pidT.calculate(-curT);

            x_rotated = x * Math.cos(heading) - y * Math.sin(heading);
            y_rotated = x * Math.sin(heading) + y * Math.cos(heading);

            // x, y, theta input mixing
            lfSpeed = x_rotated - y_rotated;
            lbSpeed = x_rotated + y_rotated;
            rfSpeed = x_rotated + y_rotated;
            rbSpeed = x_rotated - y_rotated;

            lfTemp = abs(lfSpeed);
            lbTemp = abs(lbSpeed);
            rfTemp = abs(rfSpeed);
            rbTemp = abs(rbSpeed);

            maxSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if(maxSpeed > 1) {
                lfSpeed /= maxSpeed;
                lbSpeed /= maxSpeed;
                rfSpeed /= maxSpeed;
                rbSpeed /= maxSpeed;
            }

//            lfSpeed -= t;
//            lbSpeed -= t;
//            rfSpeed += t;
//            rbSpeed += t;
//
//            lfTemp = abs(lfSpeed);
//            lbTemp = abs(lbSpeed);
//            rfTemp = abs(rfSpeed);
//            rbTemp = abs(rbSpeed);
//
//            maxSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));
//
//            if(maxSpeed > 1) {
//                lfSpeed /= maxSpeed;
//                lbSpeed /= maxSpeed;
//                rfSpeed /= maxSpeed;
//                rbSpeed /= maxSpeed;
//            }

            drive.drive(lfSpeed * speed, rfSpeed * speed, lbSpeed * speed, rbSpeed * speed);

            if(pidX.atSetPoint() && pidY.atSetPoint() /*&& pidT.atSetPoint()*/) {
                break;
            }

            cycleTime = 1000 * (opMode.getRuntime() - lastCycle);

            opMode.telemetry.addData("Cycle Time", cycleTime);
            opMode.telemetry.addData("X", curX);
            opMode.telemetry.addData("Y", curY);
            opMode.telemetry.addData("T", curT);
            opMode.telemetry.addData("X Error", endX - curX);
            opMode.telemetry.addData("Y Error", endY - curY);
            opMode.telemetry.addData("T Error", pidT.getPositionError());
            opMode.telemetry.update();

            LiftSubsystem.updatePositional();

            if (dashTelemetry != null){
                dashTelemetry.addData("X Error", endX - curX);
                dashTelemetry.addData("X PID Output", x);
                dashTelemetry.addData("Y Error", endY - curY);
                dashTelemetry.addData("Y PID Output", y);
                //dashTelemetry.addData("T Error", pidT.getPositionError());
                //dashTelemetry.addData("T PID Output", t);
                dashTelemetry.addData("Angle", curT);
                dashTelemetry.addData("LF Speed", lfSpeed * speed);
                dashTelemetry.addData("LB Speed", rfSpeed * speed);
                dashTelemetry.addData("RF Speed", rfSpeed * speed);
                dashTelemetry.addData("RB Speed", rbSpeed * speed);
                dashTelemetry.addData("Current X", curX);
                dashTelemetry.addData("Current Y", curY);
                dashTelemetry.update();
            }

        }

        Robot.leftBack.setPower(0);
        Robot.rightBack.setPower(0);
        Robot.leftFront.setPower(0);
        Robot.rightFront.setPower(0);

    }

    public static void moveTo(double endX, double endY, double speed, boolean exact) {
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();

        double heading = moving.getHeading();
        moveTo(endX, endY, heading, speed, true, exact, null);
    }

    public static void moveTo(double endX, double endY, double speed) {
        moveTo(endX, endY, speed, true);
    }

    @Deprecated
    public static void moveToLocation(double endX, double endY, double speed) {
        if(!opMode.opModeIsActive()) {
            return;
        }

        Telemetry telemetry = null;
        if(Robot.useFTCDash) {
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

        if(Robot.useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        Robot.leftBack.setPower(speed);
        Robot.leftFront.setPower(speed);
        Robot.rightBack.setPower(speed);
        Robot.rightFront.setPower(speed);

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
            Robot.leftBack.setPower(adjustedLeftSpeed);
            Robot.leftFront.setPower(adjustedLeftSpeed);
            Robot.rightBack.setPower(adjustedRightSpeed);
            Robot.rightFront.setPower(adjustedRightSpeed);

            opMode.telemetry.addData("Distance", distance);
            opMode.telemetry.addData("Curr X", curX);
            opMode.telemetry.addData("Curr Y", curY);
            opMode.telemetry.addData("Left Odo", Robot.leftOdo.getCurrentPosition());
            opMode.telemetry.addData("Right Odo", Robot.rightOdo.getCurrentPosition());
            opMode.telemetry.update();

            if(Robot.useFTCDash) {
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

        Robot.leftBack.setPower(0);
        Robot.leftFront.setPower(0);
        Robot.rightBack.setPower(0);
        Robot.rightFront.setPower(0);

    }

    @Deprecated
    public static void moveToLocation(double endX, double endY, double speed, boolean turn) {
        if(!opMode.opModeIsActive()) {
            return;
        }

        Telemetry telemetry = null;
        if(Robot.useFTCDash) {
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

        if(Robot.useFTCDash) {
            telemetry.addData("Distance (Forward Error)", distance);
            telemetry.addData("Forward PID Output", speedAdjustment);
            telemetry.addData("Angle Deviation (Angle Error)", heading - targetAngle);
            telemetry.addData("Angle PID Output", turningAdjustment);
            telemetry.addData("Left Speed Target", adjustedLeftSpeed);
            telemetry.addData("Right Speed Target", adjustedRightSpeed);


            telemetry.update();
        }

        Robot.leftBack.setPower(speed);
        Robot.leftFront.setPower(speed);
        Robot.rightBack.setPower(speed);
        Robot.rightFront.setPower(speed);

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
            Robot.leftBack.setPower(adjustedLeftSpeed);
            Robot.leftFront.setPower(adjustedLeftSpeed);
            Robot.rightBack.setPower(adjustedRightSpeed);
            Robot.rightFront.setPower(adjustedRightSpeed);

            opMode.telemetry.addData("Distance", distance);
            opMode.telemetry.addData("Curr X", curX);
            opMode.telemetry.addData("Curr Y", curY);
            opMode.telemetry.addData("Left Odo", Robot.leftOdo.getCurrentPosition());
            opMode.telemetry.addData("Right Odo", Robot.rightOdo.getCurrentPosition());
            opMode.telemetry.update();

            if(Robot.useFTCDash) {
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

        Robot.leftBack.setPower(0);
        Robot.leftFront.setPower(0);
        Robot.rightBack.setPower(0);
        Robot.rightFront.setPower(0);

    }

    public static double getRemainingTime(double timeSinceStart) {
        return 30 - timeSinceStart;
    }

    public static double getRemainingTime(double startTime, double currentTime) {
        return getRemainingTime(currentTime - startTime);
    }

    public static void turnTo180(double speed) {
        boolean turnLeft;
        double turnTol = (float) toRadians(2);
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();
        turnLeft = moving.getHeading() >= 0;

        if(turnLeft) {
            drive.drive(-speed, speed);
        } else {
            drive.drive(speed, -speed);
        }

        while(opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            double heading = moving.getHeading();
            if(heading > PI - turnTol || heading < -(PI - turnTol)) {
                break;
            }

            double remainingDegrees = Math.abs(PI - heading);

            if(remainingDegrees < (PI / 4)) {
                //Degrees for graphing and functions because scuffed PID
                remainingDegrees = Math.toDegrees(remainingDegrees);

                double p = 0.01;
                double rampDownSpeed = Math.max(abs(p * remainingDegrees), 0.35);

                if(turnLeft) {
                    drive.drive(-rampDownSpeed, rampDownSpeed);
                } else {
                    drive.drive(rampDownSpeed, -rampDownSpeed);
                }
            }

        }

        drive.stop();
    }

    public static void turn(double angDeg, double speed) {
        double turnTol = (float) toRadians(2);
        boolean turnLeft = false;
        holOdom.updatePose();
        Pose2d moving = holOdom.getPose();
        double heading = moving.getHeading();
        double targetRad = toRadians(angDeg);

        double oppAng = heading + PI;
        oppAng = oppAng > PI ? oppAng - PI : oppAng;

        if(targetRad > heading && abs(targetRad) < abs(oppAng)) {
            turnLeft = true;
        }

        if(turnLeft) {
            drive.drive(-speed, speed);
        } else {
            drive.drive(speed, -speed);
        }

        while(opMode.opModeIsActive()) {
            holOdom.updatePose();
            moving = holOdom.getPose();
            heading = moving.getHeading();

            if(abs(targetRad - heading) < turnTol) {
                break;
            }

            opMode.telemetry.addData("Error",abs(targetRad - heading));
            opMode.telemetry.update();

            double remainingDegrees = Math.abs(targetRad - heading);

            if(remainingDegrees < (PI / 4)) {
                //Degrees for graphing and functions because scuffed PID
                remainingDegrees = Math.toDegrees(remainingDegrees);

                double p = 0.01;
                double rampDownSpeed = Math.max(abs(p * remainingDegrees), 0.35);

                if(turnLeft) {
                    drive.drive(-rampDownSpeed, rampDownSpeed);
                } else {
                    drive.drive(rampDownSpeed, -rampDownSpeed);
                }
            }

        }

        drive.stop();
    }
}
