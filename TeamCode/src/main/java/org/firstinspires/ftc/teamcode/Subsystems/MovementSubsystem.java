package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.PoseSupply.ODOMETRY;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armAngle;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.armExtend;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.base;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.gripper;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.viperTouch;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.ARMPICKUPANGLE;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFD;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFI;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.NEWPIDFP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.TurnPIDP;
import static org.firstinspires.ftc.teamcode.Tuning.tuningConstants2023.odoUseIMU;
import static java.lang.Math.abs;
import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MovementSubsystem {
    private final HoloDrivetrainSubsystem holoDrivetrain;

    private final LinearOpMode opMode;
    private Telemetry telemetry;
    private final HolonomicOdometry holOdom;
    private final CameraSubsystem cameraSubsystem;
    private final ArmSubsystem armSubsystem;

    private final IMU imu;

    private static final double precision = 0.4;
    private boolean apriltagSyncRequested = false;

    private final double V1 = 0.1, V2 = 0.999;
    /**
     * Distance from point to begin slowing down
     */
    private final double D1 = 0.0, D2 = 6.0;
    private final double A = V1 / (1 - V1);
    private final double K = (1 / D2) * Math.log(V2 / (A * (1 / V2)));

    public static final Runnable GRAB_PIXEL_AUTO = () -> {
        int yes = 14;
    };

    public MovementSubsystem(HoloDrivetrainSubsystem holoDrivetrain, HolonomicOdometry holOdom, LinearOpMode OpMode, CameraSubsystem cameraSubsystem, ArmSubsystem armSubsystem, IMU imu) {
        this.holoDrivetrain = holoDrivetrain;
        this.holOdom = holOdom;
        this.opMode = OpMode;
        this.telemetry = OpMode.telemetry;
        this.cameraSubsystem = cameraSubsystem;
        this.armSubsystem = armSubsystem;
        this.imu = imu;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }


    /**
     * Move to a position
     *
     * @param poseSupply The Reference to Target When Moving
     * @param x X Position relative to the poseSupply
     * @param y Y Position relative to the poseSupply
     * @param theta Rotation relative to poseSupply
     * @param maxSpeed Max motor speeds
     * @param loop Runnable to be ran in the loop
     */
    public void moveTo(@NonNull PoseSupply poseSupply, double x, double y, double theta, double maxSpeed, boolean precise, Runnable loop) {
        theta = theta;

        if (loop == null) {
            loop = () -> {};
        }

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
        double xError = 0;
        double yError = 0;
        double heading = 0;

        if (poseSupply == ODOMETRY) {
            pose = holOdom.getPose();

        } else {
            pose = cameraSubsystem.getRelativeAprilTagPose(poseSupply);
            if (pose == null) {
                //Approximate values from odometry
                Transform2d temp = holOdom.getPose().minus(poseSupply.globalPose);
                pose = new Pose2d(temp.getTranslation(), temp.getRotation());
            }
        }

        heading = -pose.getHeading();
        xError = x - pose.getX();
        yError = y - pose.getY();


        double xPID = xController.calculate(pose.getX());
        double yPID = yController.calculate(pose.getY());
        double thetaPID = turnController.calculateOutput(theta, heading);
        double xVelocity = clip(xPID, maxSpeed, -maxSpeed);
        double yVelocity = clip(yPID, maxSpeed, -maxSpeed);
        double thetaVelocity = clip(thetaPID, maxSpeed, -maxSpeed);

        ArmSubsystem.ArmState armState = ArmSubsystem.ArmState.FreeReadyTransition;
        boolean extendZeroed = false;
        double gripTime = 0;

        double maxTime = 0;

        while (opMode.opModeIsActive() && !opMode.gamepad1.a && poseSupply != ODOMETRY) {
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
            telemetry.addData("Theta PID Output", thetaPID);
            telemetry.addData("Theta Velocity", thetaVelocity);
            telemetry.addData("Max X Time", maxTime);
            telemetry.update();
        }

        double realPrecision = precise ? precision : 2;
        while ((abs(xError) > realPrecision || abs(yError) > realPrecision || turnController.getError(theta, heading) > 0.11) && opMode.opModeIsActive()) {
            holOdom.updatePose();

            telemetry.addData("April Sync Needed", apriltagSyncRequested);

            if(apriltagSyncRequested) {
                Pose2d aprilPose = cameraSubsystem.getPoseFromAprilTag();
                if(aprilPose != null) {
                    holOdom.updatePose();
                    holOdom.updatePose(aprilPose);
                    apriltagSyncRequested = false;
                }
            }

            if (poseSupply == ODOMETRY) {
                pose = holOdom.getPose();

            } else {
                pose = cameraSubsystem.getRelativeAprilTagPose(poseSupply);
                if (pose == null) {
                    //Approximate values from odometry
                    Transform2d temp = holOdom.getPose().minus(poseSupply.globalPose);
                    pose = new Pose2d(temp.getTranslation(), temp.getRotation());
                }
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

            telemetry.addData("Arm State", armState);
            telemetry.addData("Is Doing Arm Thing", loop == GRAB_PIXEL_AUTO);
            telemetry.addData("Extend Zeroed", extendZeroed);
            telemetry.addData("X", holOdom.getPose().getX());
            telemetry.addData("Y", holOdom.getPose().getY());
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

            if (loop == GRAB_PIXEL_AUTO) {
                switch (armState) {
                    case FreeReadyTransition:
                        gripper.setPosition(1);
                        armSubsystem.setWristAngle(0);

                        if (abs(armSubsystem.getAngle() - 10) < 1 && abs(armExtend.getCurrentPosition() - base) < 10 && extendZeroed) {
                            armState = ArmSubsystem.ArmState.LowerArm;
                        }

                        if (!(abs(armSubsystem.getAngle() - 10) < 1 /*Angle not in position*/)) {
                            armAngle.setPower(-0.85 * Math.signum(armSubsystem.getAngle() - 10));
                            telemetry.addData("Moving Arm", "True");
                        } else {
                            armAngle.setPower(0);
                            telemetry.addData("Moving Arm", "False");
                        }

                        if (!extendZeroed) {
                            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            armExtend.setPower(0.65);
                            if (!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */) {
                                armExtend.setPower(0);
                                armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                extendZeroed = true;
                            }
                        } else {
                            armExtend.setTargetPosition(base);
                            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armExtend.setPower(0.8);
                        }
                        break;
                    case LowerArm:
                        armAngle.setPower(-0.85);
                        if (abs(armSubsystem.getAngle() - ARMPICKUPANGLE) < 1) {
                            armAngle.setPower(0);
                            armState = ArmSubsystem.ArmState.Grip;
                        }
                        break;
                    case Grip:
                        if (gripTime == 0) {
                            gripTime = System.currentTimeMillis();
                        }
                        gripper.setPosition(0);
                        if (System.currentTimeMillis() - gripTime > 500) {
                            armState = ArmSubsystem.ArmState.RaiseArm;
                            gripTime = 0;
                        }
                        break;
                    case RaiseArm:
                        armAngle.setPower(0.85);
                        if (armSubsystem.getAngle() > 23 /*Arm is raised*/) {
                            armAngle.setPower(0);
                            armState = ArmSubsystem.ArmState.FreeMovement;
                            armAngle.setPower(0);
                            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            armExtend.setTargetPosition(armExtend.getCurrentPosition());
                            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armExtend.setPower(0.8);
                        }
                        break;
                }
            } else {
                loop.run();
            }

        }

        holoDrivetrain.stop();

        if (loop == GRAB_PIXEL_AUTO) {
            while (armState != ArmSubsystem.ArmState.FreeMovement && opMode.opModeIsActive()) {
                switch (armState) {
                    case FreeReadyTransition:
                        gripper.setPosition(1);
                        armSubsystem.setWristAngle(0);

                        if (abs(armSubsystem.getAngle() - 10) < 1 && abs(armExtend.getCurrentPosition() - base) < 10 && extendZeroed) {
                            armState = ArmSubsystem.ArmState.LowerArm;
                        }

                        if (!(abs(armSubsystem.getAngle() - 10) < 1 /*Angle not in position*/)) {
                            armAngle.setPower(-0.85 * Math.signum(armSubsystem.getAngle() - 10));
                            telemetry.addData("Moving Arm", "True");
                        } else {
                            armAngle.setPower(0);
                            telemetry.addData("Moving Arm", "False");
                        }

                        if (!extendZeroed) {
                            if (!viperTouch.getState() /* && armExtend.getCurrentPosition() != 0 */) {
                                armExtend.setPower(0);
                                armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                extendZeroed = true;
                            }
                            armExtend.setPower(0.65);
                        } else {
                            armExtend.setTargetPosition(base);
                            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armExtend.setPower(0.8);
                        }
                        break;
                    case LowerArm:
                        armAngle.setPower(-0.85);
                        if (abs(armSubsystem.getAngle() - ARMPICKUPANGLE) < 1) {
                            armAngle.setPower(0);
                            armState = ArmSubsystem.ArmState.Grip;
                        }
                        break;
                    case Grip:
                        if (gripTime == 0) {
                            gripTime = System.currentTimeMillis();
                        }
                        gripper.setPosition(0);
                        if (System.currentTimeMillis() - gripTime > 500) {
                            armState = ArmSubsystem.ArmState.RaiseArm;
                            gripTime = 0;
                        }
                        break;
                    case RaiseArm:
                        armAngle.setPower(0.85);
                        if (armSubsystem.getAngle() > 23 /*Arm is raised*/) {
                            armAngle.setPower(0);
                            armState = ArmSubsystem.ArmState.FreeMovement;
                            armAngle.setPower(0);
                            armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            armExtend.setTargetPosition(armExtend.getCurrentPosition());
                            armExtend.setPower(0);
                        }
                        break;
                }
            }
        }
    }

    public void moveTo(@NonNull PoseSupply poseSupply, double x, double y, double theta, double maxSpeed) {
        moveTo(poseSupply, x, y, theta, maxSpeed, true,null);
    }

    public void moveTo(@NonNull PoseSupply poseSupply, double x, double y, double theta, double maxSpeed, boolean precise) {
        moveTo(poseSupply, x, y, theta, maxSpeed, precise,null);
    }

    @Deprecated
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

        while (opMode.opModeIsActive() && !opMode.gamepad1.a) {
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

        while (/*(abs(xError) > precision || abs(yError) > precision || turnController.getError(theta, heading) > 0.05) && */ opMode.opModeIsActive()) {
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

            if (maxMotorSpeed > 1) {
                lfSpeed /= maxMotorSpeed;
                lbSpeed /= maxMotorSpeed;
                rfSpeed /= maxMotorSpeed;
                rbSpeed /= maxMotorSpeed;
            }

            //holoDrivetrain.drive(-lfSpeed, -rfSpeed, -lbSpeed, -rbSpeed);
            holoDrivetrain.drive(-rbSpeed, -lbSpeed, -rfSpeed, -lfSpeed);

            if (opMode.gamepad1.b) {
                break;
            }
        }

        holoDrivetrain.stop();
    }


    private double clip(double input, double max, double min) {
        if (input > max) {
            return max;
        } else return Math.max(input, min);
    }

    public void requestApriltagSync() {
        apriltagSyncRequested = true;
//        if(!cameraSubsystem.isPortalStreaming()) {
//            cameraSubsystem.restartCamera();
//        }
    }

    public void cancelApriltagSync() {
        apriltagSyncRequested = false;
    }
}
