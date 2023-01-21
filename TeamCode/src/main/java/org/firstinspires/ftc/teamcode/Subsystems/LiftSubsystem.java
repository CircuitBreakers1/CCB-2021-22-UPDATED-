package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class LiftSubsystem extends Thread {
    private static DcMotor armLiftEncoder;
    private static DcMotor armLiftNoEncoder;
    private static DigitalChannel touch;
    private static LinearOpMode opMode;

    private static boolean manualControl = false;

    private static final double cpr = 753.2 / 4;
    private static final double spoolDiameter = 1.88976;
    private static final double spoolCircumference = Math.PI * spoolDiameter;
    private static final int cascadeMultiplier = 3;

    public static void init(DcMotor armLiftEncoder, Motor armLiftNoEncoder, DigitalChannel touch, LinearOpMode op) {
        LiftSubsystem.armLiftEncoder = armLiftEncoder;
        LiftSubsystem.armLiftNoEncoder = armLiftNoEncoder.motor;

        LiftSubsystem.opMode = op;

        LiftSubsystem.touch = touch;

        LiftSubsystem.armLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftSubsystem.armLiftNoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets manual control of the arm and automatically switches in and out of manual control
     * @param power The value of the stick
     */
    public static void manualControl(double power) {
        if (power != 0) {
            if(!manualControl) {
                armLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armLiftNoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                manualControl = true;
            }
            armLiftNoEncoder.setPower(power);
            armLiftEncoder.setPower(power);
        } else if (manualControl) {
            if(armLiftEncoder.getCurrentPosition() > 30) {
                armLiftEncoder.setTargetPosition(armLiftEncoder.getCurrentPosition());
                armLiftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLiftEncoder.setPower(1);
                armLiftNoEncoder.setPower(armLiftEncoder.getPower());
                manualControl = false;
            } else {
                armLiftEncoder.setPower(0);
                armLiftEncoder.setPower(0);
            }
        }
    }

    public static void moveToPosition(double inches) {
        setPosition((int) (inches * cpr * cascadeMultiplier / spoolCircumference));
    }

    public static void setPosition(int position) {
        armLiftEncoder.setTargetPosition(position);
        armLiftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manualControl = false;
        updatePositional();
    }

    public static void setTarget(@NonNull LiftTarget target) {
        setPosition(target.getPosition());
    }

    public static double getArmInches() {
        return (spoolCircumference * armLiftEncoder.getCurrentPosition()) / (cpr * cascadeMultiplier);
    }

    /**
     * <b>Run always in loops <u> or run the thread</u> as it prevents the motors from interfering with eachother </b> <br>
     * Updates the encoderless motor to match the encoder motor.
     * Automatically checks if motor is in automatic mode. <br>
     * Reduces speed if arm is travelling downwards. <br>
     * Resets position if arm is on sensor
     */
    public static void updatePositional() {
        if (!manualControl) {
            if (armLiftEncoder.getCurrentPosition() < 30 && armLiftEncoder.getTargetPosition() < 30) {
                armLiftEncoder.setPower(0);
                armLiftNoEncoder.setPower(0);
                manualControl = true;
            }else if(abs(armLiftEncoder.getCurrentPosition()) > abs(armLiftEncoder.getTargetPosition())) {
                armLiftEncoder.setPower(0.5);
            } else {
                armLiftEncoder.setPower(1);
            }
            armLiftNoEncoder.setPower(armLiftEncoder.getPower());
        } else if(!touch.getState() && armLiftEncoder.getCurrentPosition() != 0) {
            armLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftNoEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armLiftNoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armLiftEncoder.setPower(0);
            armLiftNoEncoder.setPower(0);
        }
//        opMode.telemetry.addData("Target", armLiftEncoder.getTargetPosition());
//        opMode.telemetry.addData("Current", armLiftEncoder.getCurrentPosition());
//        opMode.telemetry.update();
    }

    public static boolean isAtTarget() {
        int target = armLiftEncoder.getTargetPosition();
        int current = armLiftEncoder.getCurrentPosition();
        //If the current is within 5% of the target return true
        return abs(current - target) < 50;
    }

    /**
     * The thread allows the arm to move smoothly with linear code. <br>
     * The thread needs to be manually stopped when the arm is no longer needed.
     */
    @SuppressWarnings("InfiniteLoopStatement")
    @Override
    public void run() {
        while (true) {
            updatePositional();
            try {
                //noinspection BusyWait
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Stops the thread and disables motors
     */
    public void safestopThread() {
        this.interrupt();
        armLiftEncoder.setPower(0);
        armLiftNoEncoder.setPower(0);
        armLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftNoEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum LiftTarget {
        Min(0),
        Intake(400),
        ConeStack(800),
        Ground(240),
        Low(1060),
        Medium(1350),
        High(1975),
        ;

        private final int position;

        LiftTarget(int i) {
            this.position = i;
        }

        public int getPosition() {
            return position;
        }
    }
}
