package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftSubsystem {
    //Lift Motors
    private DcMotor leftLift;
    private DcMotor rightLift;

    //Servos
    private Servo thru;
    private Servo rotation;
    private Servo leftFinger;
    private Servo rightFinger;

    //Sensors
    private DigitalChannel liftTouch;

    //Memory
    private LiftStates liftState = LiftStates.BASE;

    //Enum Set points
    enum ThruPositions {
        BASE(0.415), BACKDROP(0.365);
        private double position;
        ThruPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    enum RotationPositions {
        BASE(0.605), VERTICAL(0.605), HORIZONTAL(0.547);
        private double position;
        RotationPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    enum FingerPositions {
        OPEN(0.0), CLOSED(0.0);
        private double position;
        FingerPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    enum LiftStates {
        BASE, BASE_TO_ACTIVE, ACTIVE, ACTIVE_TO_BASE
    }

    public LiftSubsystem(DcMotor leftLift, DcMotor rightLift, Servo thru, Servo rotation, Servo leftFinger, Servo rightFinger, DigitalChannel liftTouch) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.thru = thru;
        this.rotation = rotation;
        this.leftFinger = leftFinger;
        this.rightFinger = rightFinger;
        this.liftTouch = liftTouch;

        try {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(!liftTouch.getState()) {
                leftLift.setPower(-0.5);
                rightLift.setPower(-0.5);
            }

            leftLift.setPower(0);
            rightLift.setPower(0);
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (NullPointerException ignored) {}
    }

    public void simpleLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public LiftStates assistedControl(Gamepad gamepad) {
        return null;
    }
}
