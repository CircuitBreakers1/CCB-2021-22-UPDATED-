package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmSubsystem {
    private final Servo wrist;
    private final Servo gripper;
    private final DcMotor armAngle;
    private final DcMotor armLength;
    private final AnalogInput armEncoder;


    //Rest Angle: 5, Length: -64
    //Pickup Angle: 2 Length: -64
    //Free: 23

    private static final double OFFSET = 420.76;
    private static final double WRIST_MIN = 17.0, WRIST_MAX = 90.0;

    public ArmSubsystem(Servo wrist, Servo gripper, DcMotor armAngle, AnalogInput armEncoder, DcMotor armLength) {
        this.wrist = wrist;
        this.gripper = gripper;
        this.armAngle = armAngle;
        this.armEncoder = armEncoder;
        this.armLength = armLength;
    }

    public double getAngle() {
        double v = armEncoder.getVoltage();
        final double a = 400 * pow(v, 2) - 440 * v + 363;
        double t = -(135 * (sqrt(a) + 20 * sqrt(3) * v - 33 * sqrt(3)))
                    /(11 * sqrt(3) - sqrt(a));
        return t - OFFSET;
    }

    public void setWristAngle(double angle) {
        if(angle < WRIST_MIN) {
            angle = WRIST_MIN;
            RobotLog.ee("ArmSubsystem", "Wrist angle too low");
        } else if(angle > WRIST_MAX) {
            RobotLog.ee("ArmSubsystem", "Wrist angle too high");
            angle = WRIST_MAX;
        }

        double position = (angle - WRIST_MIN) / (WRIST_MAX - WRIST_MIN);

        wrist.setPosition(position);
    }

    public void setAbsoluteWristAngle(double angle) {
        setWristAngle(90 - getAngle() - angle);
    }

    public enum ArmState {
        Ready, LowerArm, Grip, RaiseArm, FreeMovement, FreeReadyTransition
    }
}

