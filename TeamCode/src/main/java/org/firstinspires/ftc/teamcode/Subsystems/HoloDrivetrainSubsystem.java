package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class HoloDrivetrainSubsystem {
    private final MotorEx leftFront;
    private final MotorEx rightFront;
    private final MotorEx leftBack;
    private final MotorEx rightBack;

    public HoloDrivetrainSubsystem(MotorEx lf, MotorEx rf, MotorEx lb, MotorEx rb) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;
    }

    public void drive(double lf, double rf, double lb, double rb) {
        leftFront.set(lf);
        rightFront.set(rf);
        leftBack.set(lb);
        rightBack.set(rb);
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    /**
     * Smooth Deceleration for Evan.
     * Tune to feel, too small max D will feel unresponsive, so much will feel abrupt.
     * Always call in loop to achieve desired power
     * @param motor The motor to set power to
     * @param targetPower The power to set the motor to
     */
    public void smoothDecelerate(MotorEx motor, double targetPower) {
        double maxDecelerate = 0.25;
        double currentPower = motor.get();
        if (abs(targetPower) > abs(currentPower) - abs(maxDecelerate)) {
            motor.set(targetPower);
        } else {
            motor.set(currentPower + (maxDecelerate * (-1 * signum(currentPower))));
        }
    }

    public void smoothDrive(double x, double y, double turn) {
        double leftFrontPower = y + x + turn;
        double leftBackPower = y - x + turn;
        double rightFrontPower = y - x - turn;
        double rightBackPower = y + x - turn;

        double maxPower = Math.max(Math.max(abs(leftFrontPower), abs(leftBackPower)),
                Math.max(abs(rightFrontPower), abs(rightBackPower)));

        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        smoothDecelerate(leftFront, leftFrontPower);
        smoothDecelerate(leftBack, leftBackPower);
        smoothDecelerate(rightFront, rightFrontPower);
        smoothDecelerate(rightBack, rightBackPower);
    }
}
