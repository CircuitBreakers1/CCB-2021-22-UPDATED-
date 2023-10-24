package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.LOGCATTAG;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants.NEWPIDFD;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants.NEWPIDFI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants.NEWPIDFP;
import static java.lang.Math.abs;
import static java.lang.Math.exp;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MovementSubsystem {
    private final HoloDrivetrainSubsystem holoDrivetrain;

    private final LinearOpMode opMode;
    private Telemetry telemetry;
    private final HolonomicOdometry holOdom;

    private static final double precision = 0.5;

    private final double V1 = 0.1, V2 = 0.999;
    /**
     * Distance from point to begin slowing down
     */
    private final double D1 = 0.0, D2 = 6.0;
    private final double A = V1 / (1 - V1);
    private final double K = (1 / D2) * Math.log(V2 / (A * (1 / V2)));

    public MovementSubsystem(HoloDrivetrainSubsystem holoDrivetrain, HolonomicOdometry holOdom, LinearOpMode OpMode) {
        this.holoDrivetrain = holoDrivetrain;
        this.holOdom = holOdom;
        this.opMode = OpMode;
        this.telemetry = OpMode.telemetry;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public void moveToPose(double x, double y, double theta, double maxSpeed) {
        PIDController xController = new PIDController(NEWPIDFP, NEWPIDFI, NEWPIDFD);
        xController.setSetPoint(x);
        PIDController yController = new PIDController(NEWPIDFP, NEWPIDFI, NEWPIDFD);
        yController.setSetPoint(y);
        //PIDController turnController = new PIDController(0.1, 0, 0);

        holOdom.updatePose();
        Pose2d pose = holOdom.getPose();
        double xError = x - pose.getX();
        double yError = y - pose.getY();
        double heading = pose.getHeading();

        double xPID = xController.calculate(pose.getX());
        double yPID = yController.calculate(pose.getY());
        double xVelocity = clip(xPID, maxSpeed, -maxSpeed);
        double yVelocity = clip(yPID, maxSpeed, -maxSpeed);

        double maxX = xVelocity;
        double maxTime = 0;

        while(opMode.opModeIsActive() && !opMode.gamepad1.a) {
            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();
        }

        while((/*abs(xError) > precision || abs(yError) > precision &&*/ opMode.opModeIsActive())) {
            pose = holOdom.getPose();
            xError = x - pose.getX();
            yError = y - pose.getY();
            heading = pose.getHeading();

            xPID = xController.calculate(pose.getX());
            yPID = yController.calculate(pose.getY());
            xVelocity = clip(xPID, maxSpeed, -maxSpeed);
            yVelocity = clip(yPID, maxSpeed, -maxSpeed);

            if(xVelocity > maxX) {
                maxX = xVelocity;
                maxTime = opMode.getRuntime() - maxTime;
            }

            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();

            double x_rotated = xVelocity * Math.cos(heading) - yVelocity * Math.sin(heading);
            double y_rotated = xVelocity * Math.sin(heading) + yVelocity * Math.cos(heading);

            double lfSpeed = x_rotated - y_rotated;
            double lbSpeed = x_rotated + y_rotated;
            double rfSpeed = x_rotated + y_rotated;
            double rbSpeed = x_rotated - y_rotated;

            double lfTemp = abs(lfSpeed);
            double lbTemp = abs(lbSpeed);
            double rfTemp = abs(rfSpeed);
            double rbTemp = abs(rbSpeed);

            double maxMotorSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if(maxMotorSpeed > 1) {
                lfSpeed /= maxMotorSpeed;
                lbSpeed /= maxMotorSpeed;
                rfSpeed /= maxMotorSpeed;
                rbSpeed /= maxMotorSpeed;
            }

            holoDrivetrain.drive(lfSpeed, rfSpeed, lbSpeed, rbSpeed);

            if(opMode.gamepad1.a) {
                break;
            }
        }

        holoDrivetrain.stop();
    }

    public void moveTo(double x, double y, double theta, double maxSpeed) {
        RobotLog.d(LOGCATTAG + "Moving to " + x + ", " + y + ", " + theta);
        double distance = Math.hypot(x - holOdom.getPose().getX(), y - holOdom.getPose().getY());
        RobotLog.v(LOGCATTAG + "Distance: " + distance);

        double xSpeed, ySpeed, turnSpeed, xDistance, yDistance, turnDistance, heading;
        Pose2d pose;

    while(opMode.opModeIsActive() && !opMode.gamepad1.a) {
        telemetry.addData("X Distance", 0);
        telemetry.addData("Y Distance", 0);
        telemetry.addData("X Speed", 0);
        telemetry.addData("Y Speed", 0);
        }

        while(distance > precision && opMode.opModeIsActive()) {
            holOdom.updatePose();
            pose = holOdom.getPose();
            xDistance = x - pose.getX();
//            yDistance = y - pose.getY();
            yDistance = 0;
            heading = pose.getHeading();
            turnDistance = theta - heading;
            distance = Math.hypot(xDistance, yDistance);
            xSpeed = getSpeedFromDistance(xDistance);
            ySpeed = getSpeedFromDistance(yDistance);


            telemetry.addData("X Distance", xDistance);
            telemetry.addData("Y Distance", yDistance);
            telemetry.addData("X Speed", xSpeed);
            telemetry.addData("Y Speed", ySpeed);
//            telemetry.addData("Turn Speed", turnSpeed);

            telemetry.addData("Turn Distance", turnDistance);
            telemetry.addData("Heading", heading);
            telemetry.addData("Distance", distance);
            telemetry.update();

            if(xSpeed > maxSpeed) {
                xSpeed = maxSpeed;
            }
            if(ySpeed > maxSpeed) {
                ySpeed = maxSpeed;
            }
            if(abs(ySpeed) < 0.001) {
                ySpeed = 0;
            }
            if(abs(xSpeed) < 0.001) {
                xSpeed = 0;
            }

            double x_rotated = xSpeed * Math.cos(heading) - ySpeed * Math.sin(heading);
            double y_rotated = xSpeed * Math.sin(heading) + ySpeed * Math.cos(heading);

            double lfSpeed = x_rotated - y_rotated;
            double lbSpeed = x_rotated + y_rotated;
            double rfSpeed = x_rotated + y_rotated;
            double rbSpeed = x_rotated - y_rotated;

            double lfTemp = abs(lfSpeed);
            double lbTemp = abs(lbSpeed);
            double rfTemp = abs(rfSpeed);
            double rbTemp = abs(rbSpeed);

            double maxMotorSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if(maxMotorSpeed > 1) {
                lfSpeed /= maxMotorSpeed;
                lbSpeed /= maxMotorSpeed;
                rfSpeed /= maxMotorSpeed;
                rbSpeed /= maxMotorSpeed;
            }

            holoDrivetrain.drive(lfSpeed, rfSpeed, lbSpeed, rbSpeed);

            RobotLog.v(LOGCATTAG + "X Speed: " + xSpeed + ", Y Speed: " + ySpeed + ", X Distance: " + xDistance + ", Y Distance: " + yDistance + ", Heading: " + heading + ", Turn Distance: " + turnDistance + ", Distance: " + distance);
        }
        RobotLog.d(LOGCATTAG + "Finished moving to " + x + ", " + y + ", " + theta + ". Actual Position: " + holOdom.getPose().toString());
        holoDrivetrain.stop();
    }

    private double getSpeedFromDistance(double distance) {
        if(abs(distance) < precision) {
            return 0;
        }
        int sign = distance < 0 ? -1 : 1;

        distance = abs(distance);

        double top = A * exp(K * distance);
        double bottom = 1 + (top);

        return sign * top / bottom;
    }

    private double clip(double input, double max, double min) {
        if(input > max) {
            return max;
        } else return Math.max(input, min);
    }
}
