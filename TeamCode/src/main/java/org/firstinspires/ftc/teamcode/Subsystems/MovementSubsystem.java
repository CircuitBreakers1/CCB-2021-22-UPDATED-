package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.LOGCATTAG;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFD;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.TurnPIDP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.odoUseIMU;
import static java.lang.Math.abs;
import static java.lang.Math.exp;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TeleOP.PAngleController;

public class MovementSubsystem {
    private final HoloDrivetrainSubsystem holoDrivetrain;

    private final LinearOpMode opMode;
    private Telemetry telemetry;
    private final HolonomicOdometry holOdom;

    private final IMU imu;

    private static final double precision = 0.25;

    private final double V1 = 0.1, V2 = 0.999;
    /**
     * Distance from point to begin slowing down
     */
    private final double D1 = 0.0, D2 = 6.0;
    private final double A = V1 / (1 - V1);
    private final double K = (1 / D2) * Math.log(V2 / (A * (1 / V2)));

    public MovementSubsystem(HoloDrivetrainSubsystem holoDrivetrain, HolonomicOdometry holOdom, LinearOpMode OpMode, IMU imu) {
        this.holoDrivetrain = holoDrivetrain;
        this.holOdom = holOdom;
        this.opMode = OpMode;
        this.telemetry = OpMode.telemetry;
        this.imu = imu;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public enum PoseSupplyMethod {
        ODOMETRY(-1),
        APRILTAG_BLUE_LEFT(1), APRILTAG_BLUE_MIDDLE(2), APRILTAG_BLUE_RIGHT(3),
        APRILTAG_RED_LEFT(4), APRILTAG_RED_MIDDLE(5), APRILTAG_RED_RIGHT(6);

        int id;

        PoseSupplyMethod(int id) {
            this.id = id;
        }
    }

    public void moveTo(PoseSupplyMethod poseSupply, double x, double y, double theta, double maxSpeed) {
        theta = -theta;

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

        holOdom.updatePose();
        Pose2d pose = holOdom.getPose();
        double xError = x - pose.getX();
        double yError = y - pose.getY();
        double heading = odoUseIMU ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : -pose.getHeading();
//        double thetaAdjusted = theta;
//        int thetaAdj;
//        if(theta - heading < PI * -2) {
//            thetaAdjusted += 2 * PI;
//            thetaAdj = 1;
//        } else if(theta - heading > PI * 2) {
//            thetaAdjusted -= 2 * PI;
//            thetaAdj = -1;
//        } else {
//            thetaAdjusted = theta;
//            thetaAdj = 0;
//        }

        //turnController.setSetPoint(thetaAdjusted);

        double xPID = xController.calculate(pose.getX());
        double yPID = yController.calculate(pose.getY());
        double thetaPID = turnController.calculateOutput(theta, heading);
        double xVelocity = clip(xPID, maxSpeed, -maxSpeed);
        double yVelocity = clip(yPID, maxSpeed, -maxSpeed);
        double thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);

        double maxTimeStamp = opMode.getRuntime();
        double maxTime = 0;
        boolean xInc = false;
        double prevX = xVelocity;

        while(opMode.opModeIsActive() && !opMode.gamepad1.a) {
            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("IMU Angle", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Odo Angle", -pose.getHeading());
            telemetry.addData("Theta Error", turnController.getError(theta, heading));
            telemetry.addData("Theta Error 2", theta - heading);
            //telemetry.addData("Theta Error Adjusted", thetaAdjusted - heading);
            telemetry.addData("Theta PID Output", thetaPID);
            //telemetry.addData("Theta Adjustment", thetaAdj);
            telemetry.addData("Theta Velocity", thetaVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();
        }

        while((abs(xError) > precision || abs(yError) > precision || turnController.getError(theta, heading) > 0.05) && opMode.opModeIsActive()) {
            holOdom.updatePose();
            pose = holOdom.getPose();
            xError = x - pose.getX();
            yError = y - pose.getY();
            heading = odoUseIMU ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : -pose.getHeading();


            xPID = xController.calculate(pose.getX());
            yPID = yController.calculate(pose.getY());
            thetaPID = turnController.calculateOutput(theta, heading);
            xVelocity = clip(xPID, maxSpeed, -maxSpeed);
            yVelocity = clip(yPID, maxSpeed, -maxSpeed);
            thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);


            System.out.println("Theta Velocity: " + thetaVelocity + " Time: " + opMode.getRuntime());

            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("IMU Angle", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
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

            double lfSpeed = x_rotated - y_rotated + thetaVelocity;
            double lbSpeed = x_rotated + y_rotated + thetaVelocity;
            double rfSpeed = x_rotated + y_rotated - thetaVelocity;
            double rbSpeed = x_rotated - y_rotated - thetaVelocity;

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

            if(opMode.gamepad1.b) {
                break;
            }
        }

        holoDrivetrain.stop();
    }




    public void moveToPose(double x, double y, double theta, double maxSpeed) {
        //theta = -theta;

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

        holOdom.updatePose();
        Pose2d pose = holOdom.getPose();
        double xError = x - pose.getX();
        double yError = y - pose.getY();
        double heading = odoUseIMU ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : pose.getHeading();

        double xPID = xController.calculate(pose.getX());
        double yPID = yController.calculate(pose.getY());
        double thetaPID = turnController.calculateOutput(theta, heading);
        double xVelocity = clip(xPID, maxSpeed, -maxSpeed);
        double yVelocity = clip(yPID, maxSpeed, -maxSpeed);
        double thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);

        double maxTimeStamp = opMode.getRuntime();
        double maxTime = 0;
        boolean xInc = false;
        double prevX = xVelocity;

        while(opMode.opModeIsActive() && !opMode.gamepad1.a) {
            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("IMU Angle", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Odo Angle", -pose.getHeading());
            telemetry.addData("Theta Error", turnController.getError(theta, heading));
            telemetry.addData("Theta Error 2", theta - heading);
            //telemetry.addData("Theta Error Adjusted", thetaAdjusted - heading);
            telemetry.addData("Theta PID Output", thetaPID);
            //telemetry.addData("Theta Adjustment", thetaAdj);
            telemetry.addData("Theta Velocity", thetaVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();
        }

        while(/*(abs(xError) > precision || abs(yError) > precision || turnController.getError(theta, heading) > 0.05) &&*/ opMode.opModeIsActive()) {
            holOdom.updatePose();
            pose = holOdom.getPose();
            xError = x - pose.getX();
            yError = y - pose.getY();
            heading = odoUseIMU ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : pose.getHeading();

//            thetaAdjusted = theta;
//            if(theta - heading < PI) {
//                thetaAdjusted += 2 * PI;
//                thetaAdj = 1;
//            } else if(theta - heading > PI) {
//                thetaAdjusted -= 2 * PI;
//                thetaAdj = -1;
//            } else {
//                thetaAdjusted = theta;
//                thetaAdj = 0;
//            }
//
//            turnController.setSetPoint(thetaAdjusted);

            xPID = xController.calculate(pose.getX());
            yPID = yController.calculate(pose.getY());
            thetaPID = turnController.calculateOutput(theta, heading);
            xVelocity = clip(xPID, maxSpeed, -maxSpeed);
            yVelocity = clip(yPID, maxSpeed, -maxSpeed);
            thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);


            System.out.println("X Velocity: " + xVelocity + " Time: " + opMode.getRuntime());

            telemetry.addData("X Distance", xError);
            telemetry.addData("Y Distance", yError);
            telemetry.addData("X PID Output", xPID);
            telemetry.addData("Y PID Output", yPID);
            telemetry.addData("X Velocity", xVelocity);
            telemetry.addData("Y Velocity", yVelocity);
            telemetry.addData("Theta (" + (odoUseIMU ? "IMU" : "Odo") + ")", heading);
            telemetry.addData("IMU Angle", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
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

            double lfSpeed = x_rotated - y_rotated + thetaVelocity;
            double lbSpeed = x_rotated + y_rotated + thetaVelocity;
            double rfSpeed = x_rotated + y_rotated - thetaVelocity;
            double rbSpeed = x_rotated - y_rotated - thetaVelocity;

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

            if(opMode.gamepad1.b) {
                break;
            }
        }
        
        holoDrivetrain.stop();
    }





    private double clip(double input, double max, double min) {
        if(input > max) {
            return max;
        } else return Math.max(input, min);
    }
}
