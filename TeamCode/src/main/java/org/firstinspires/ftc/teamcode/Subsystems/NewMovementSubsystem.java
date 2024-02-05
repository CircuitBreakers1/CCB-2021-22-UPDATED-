package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tuning.OldAutoTuning.pauseForTuning;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFD;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.TurnPIDP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.odoUseIMU;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewMovementSubsystem {
    private final HoloDrivetrainSubsystem holoDrivetrain;
    private final LinearOpMode opMode;
    private Telemetry telemetry;
    private final HolonomicOdometry holOdom;
    private final CameraSubsystem cameraSubsystem;
    private final PixelSubsystem pixelSubsystem;
    private static final double precision = 0.4;
    private boolean apriltagSyncRequested = false;
    //Minimum inches to travel in a given time period to avoid timing out
    private static final double minInches = 0.25;
    private static final double minDegrees = 5;
    private static final double minTime = 2;

    public NewMovementSubsystem(HoloDrivetrainSubsystem holoDrivetrain, HolonomicOdometry holOdom, LinearOpMode OpMode, CameraSubsystem cameraSubsystem, PixelSubsystem pixelSubsystem) {
        this.holoDrivetrain = holoDrivetrain;
        this.holOdom = holOdom;
        this.opMode = OpMode;
        this.telemetry = OpMode.telemetry;
        this.cameraSubsystem = cameraSubsystem;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        this.pixelSubsystem = pixelSubsystem;
    }


    /**
     * Move to a position
     * @param x X Position relative to the poseSupply
     * @param y Y Position relative to the poseSupply
     * @param theta Rotation relative to poseSupply
     * @param maxSpeed Max motor speeds
     * @param loop Runnable to be ran in the loop
     */
    public void moveTo(double x, double y, double theta, double maxSpeed, boolean precise, Runnable loop) {
        //Ku = 0.39, Tu = 0.719
        //Z-N Values: Kp = 0.078, Ki = 0.217, Kd = 0.0185
        PIDController xController = new PIDController(NEWPIDFP, NEWPIDFI, NEWPIDFD);
        xController.setSetPoint(x);
        PIDController yController = new PIDController(NEWPIDFP, NEWPIDFI, NEWPIDFD);
        yController.setSetPoint(y);

        //Ku = 8.9 Tu = 0.13347
        //Z-N Values: Kp = 1.78, Ki = 26.7, Kd = 0.0784
        //Better Kp = 0.6, Ki, Kd = 0
        //PIDController turnController = new PIDController(TurnPIDP, TurnPIDI, TurnPIDD);
        PAngleController turnController = new PAngleController(TurnPIDP);


        if(apriltagSyncRequested) {
            Pose2d aprilPose = cameraSubsystem.getPoseFromAprilTag();
            if(aprilPose != null) {
                holOdom.updatePose(aprilPose);
                apriltagSyncRequested = false;
            } else {
                holOdom.updatePose();
            }
        } else {
            holOdom.updatePose();
        }

        Pose2d pose;
        Pose2d lastPose;
        double timestamp = System.currentTimeMillis();
        double xError;
        double yError;
        double heading;
        double translationDistance = 0;
        double rotationDistance = 0;


        pose = holOdom.getPose();
        lastPose = pose;

        heading = -pose.getHeading();
        xError = x - pose.getX();
        yError = y - pose.getY();


        double xPID = xController.calculate(pose.getX());
        double yPID = yController.calculate(pose.getY());
        double thetaPID = turnController.calculateOutput(theta, heading);
        double xVelocity = clip(xPID, maxSpeed, -maxSpeed);
        double yVelocity = clip(yPID, maxSpeed, -maxSpeed);
        double thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);

        double maxTime = 0;

        while (opMode.opModeIsActive() && !opMode.gamepad1.a && pauseForTuning) {
            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("Odo Angle", -pose.getHeading());
            telemetry.addData("Theta Error", turnController.getError(theta, heading));
            telemetry.addData("Theta Error 2", theta - heading);
            telemetry.addData("Theta PID Output", thetaPID);
            telemetry.addData("Theta Velocity", thetaVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();
        }

        double realPrecision = precise ? precision : 1;
        while ((abs(xError) > realPrecision || abs(yError) > realPrecision || turnController.getError(theta, heading) > 0.11) && opMode.opModeIsActive()) {
            holOdom.updatePose();
            pixelSubsystem.runPixelSystem();

            telemetry.addData("April Sync Needed", apriltagSyncRequested);

            if(apriltagSyncRequested) {
                Pose2d aprilPose = cameraSubsystem.getPoseFromAprilTag();
                if(aprilPose != null) {
                    holOdom.updatePose();
                    holOdom.updatePose(aprilPose);
                    apriltagSyncRequested = false;
                }
            }

            pose = holOdom.getPose();

            translationDistance += Math.hypot(pose.getX() - lastPose.getX(), pose.getY() - lastPose.getY());
            rotationDistance += Math.abs(Math.toDegrees(pose.getHeading()) - Math.toDegrees(lastPose.getHeading()));

            lastPose = pose;
            double time = System.currentTimeMillis() - timestamp;
            if(time > maxTime * 1000) {
                double translationAdjust = translationDistance / (time / (maxTime * 1000));
                double rotationAdjust = rotationDistance / (time / (maxTime * 1000));

                if(translationAdjust < minInches && rotationAdjust < minDegrees) {
                    break;
                }
                translationDistance = 0;
                rotationDistance = 0;
                timestamp = System.currentTimeMillis();
            }

            heading = -pose.getHeading();
            xError = x - pose.getX();
            yError = y - pose.getY();


            xPID = xController.calculate(pose.getX());
            yPID = yController.calculate(pose.getY());
            thetaPID = turnController.calculateOutput(theta, heading);
            xVelocity = clip(xPID, maxSpeed, -maxSpeed);
            yVelocity = clip(yPID, maxSpeed, -maxSpeed);
            thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);


            System.out.println("Theta Velocity: " + thetaVelocity + " Time: " + opMode.getRuntime());

            telemetry.addData("X", holOdom.getPose().getX());
            telemetry.addData("Y", holOdom.getPose().getY());
            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("Odo Angle", -pose.getHeading());
            telemetry.addData("Theta Error", turnController.getError(theta, heading));
            telemetry.addData("Theta Error 2", theta - heading);
            //telemetry.addData("Theta Error Adjusted", thetaAdjusted - heading);
            telemetry.addData("Theta PID Output", thetaPID);
            //telemetry.addData("Theta Adjustment", thetaAdj);
            telemetry.addData("Theta Velocity", thetaVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();

            double x_rotated = xVelocity * Math.cos(heading) - yVelocity * Math.sin(heading);
            double y_rotated = xVelocity * Math.sin(heading) + yVelocity * Math.cos(heading);

            double lfSpeed = x_rotated - y_rotated - thetaVelocity;
            double lbSpeed = x_rotated + y_rotated - thetaVelocity;
            double rfSpeed = x_rotated + y_rotated + thetaVelocity;
            double rbSpeed = x_rotated - y_rotated + thetaVelocity;

            double lfTemp = abs(lfSpeed);
            double lbTemp = abs(lbSpeed);
            double rfTemp = abs(rfSpeed);
            double rbTemp = abs(rbSpeed);

            double maxMotorSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if (maxMotorSpeed > 1) {
                lfSpeed /= maxMotorSpeed;
                lbSpeed /= maxMotorSpeed;
                rfSpeed /= maxMotorSpeed;
                rbSpeed /= maxMotorSpeed;
            }

            //holoDrivetrain.drive(lfSpeed, rfSpeed, lbSpeed, rbSpeed);
            //Flipped for gaslighting purposes
            //holoDrivetrain.drive(-rbSpeed, -lbSpeed, -rfSpeed, -lfSpeed);
            holoDrivetrain.drive(-rbSpeed, -lbSpeed, -rfSpeed, -lfSpeed);

            if (opMode.gamepad1.b) {
                break;
            }
        }
        holoDrivetrain.stop();
    }

    public void moveTo(double x, double y, double theta, double maxSpeed) {
        moveTo(x, y, theta, maxSpeed, true,null);
    }

    public void moveTo(double x, double y, double theta, double maxSpeed, boolean precise) {
        moveTo(x, y, theta, maxSpeed, precise,null);
    }

    private double clip(double input, double max, double min) {
        if (input > max) {
            return max;
        } else return Math.max(input, min);
    }

    public void requestApriltagSync() {
        apriltagSyncRequested = true;
    }

    public void cancelApriltagSync() {
        apriltagSyncRequested = false;
    }
}
