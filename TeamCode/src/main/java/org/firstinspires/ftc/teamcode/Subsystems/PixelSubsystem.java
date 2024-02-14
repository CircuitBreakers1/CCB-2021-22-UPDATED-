package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ColorDetectionSubsystem.BayColor.WHITE;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.OPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.PixelSubsystem.FingerPositions.RIGHT_OPEN;
import static org.firstinspires.ftc.teamcode.Tuning.NewTuning.PDHeight;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/** @noinspection UnusedReturnValue, CommentedOutCode */
public class PixelSubsystem {
    //Lift Motors
    private final DcMotor leftLift;
    private final DcMotor rightLift;
    private final DcMotor intake;

    //Servos
    private final Servo thru;
    private final Servo rotation;
    private final Servo leftFinger;
    private final Servo rightFinger;
    private final Servo frontStage;

    //Sensors
    /** @noinspection FieldCanBeLocal, unused */
    private final DigitalChannel liftTouch;
    private final NormalizedColorSensor colorSensor;

    //Misc Objects
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private final Gamepad.RumbleEffect whiteRumble;
    private final Gamepad.RumbleEffect yellowRumble;
    private final Gamepad.RumbleEffect greenRumble;
    private final Gamepad.RumbleEffect purpleRumble;
    private final PIDController armPID;

    //Memory
    private LiftStates liftState = LiftStates.BASE;
    private IntakeState intakeState = IntakeState.OFF;
    private boolean teleOp = false;
    private boolean colorTriggered = false;
    public int pixelCount = 0;
    private double outputTimestamp = -1;
    private double flipTimestamp = -1;
    private double liftPower = 0.0;
    private boolean leftDrop = false;
    private boolean rightDrop = false;
    private int liftSetpoint = 0;
    private boolean frontStageOverride = false;
    private boolean fingerOverride = false;
    private boolean rumbleRequested = false;

    /** @noinspection FieldCanBeLocal*/ //Tuning Constants
    //TODO: Find actual values
    private final int minFlipHeight = 460;
    /** @noinspection FieldCanBeLocal*/
    private final double colorTriggerDist = 6;

    //Enum Set points
    enum ThruPositions {
        BASE(0.525), BACKDROP(0.475), PD(0.48);
        private final double position;
        ThruPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    enum RotationPositions {
        BASE(0.46), VERTICAL(0.46), HORIZONTAL(0.16);
        private final double position;
        RotationPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    public enum FingerPositions {
        LEFT_OPEN(0.99), LEFT_CLOSED(0.8), RIGHT_OPEN(0.75), RIGHT_CLOSED(0.55), OPEN(0), CLOSED(0);
        private final double position;
        FingerPositions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return position;
        }
    }

    enum LiftStates {
        BASE, AUTO_RAISE, FLIP_OUT, VERTICAL_MANUAL, HORIZONTAL_MANUAL, MIN_RAISE, FLIP_IN, BASE_RETURN,
        FULL_MANUAL,
        PD_RAISE, PD_FLIP, PD_LOWER, PD_READY
    }

    enum IntakeState {
        INPUT, UNSTABLE_OUTPUT, OFF, OUTPUT
    }


    public PixelSubsystem(DcMotor leftLift, DcMotor rightLift, DcMotor intake, Servo frontStage, Servo thru, Servo rotation, Servo leftFinger, Servo rightFinger, DigitalChannel liftTouch, NormalizedColorSensor colorSensor) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.intake = intake;
        this.thru = thru;
        this.rotation = rotation;
        this.leftFinger = leftFinger;
        this.rightFinger = rightFinger;
        this.liftTouch = liftTouch;
        this.colorSensor = colorSensor;
        this.frontStage = frontStage;


        armPID = new PIDController(10,0.04999,0);
        armPID.setTolerance(50);

        whiteRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0.75, 0, 250)
                .addStep(0, 0.75, 250)
                .build();

        yellowRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 0.75, 250)
                .addStep(0.5, 0, 250)
                .build();

        greenRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 0.75, 250)
                .addStep(0.75, 0, 250)
                .build();

        purpleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0, 0.75, 250)
                .addStep(1, 0, 250)
                .build();
    }

    public void initGamepadsForTeleOP(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        teleOp = true;
    }

    public void setTeleOp() {
        teleOp = true;
    }

    public void initArm() {
        setThroughPosition(ThruPositions.BASE);
        thru.setPosition(ThruPositions.BASE.getPosition());
        setRotationPosition(RotationPositions.BASE);
        double timestamp = System.currentTimeMillis();
        while(System.currentTimeMillis() - timestamp < 100) {
        }

        while(liftTouch.getState()) {
            leftLift.setPower(-0.5);
            rightLift.setPower(-0.5);
        }

        leftLift.setPower(0);
        rightLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Deprecated
    public void simpleLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    double intakeScheduler = -1;
    public void runPixelSystem() {
        //Intake
        if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < colorTriggerDist) {
            if(!colorTriggered) {
                pixelCount++;
                if(pixelCount >= 2) {
                    intakeScheduler = System.currentTimeMillis() + 750;
                }
                rumbleRequested = true;
                if(teleOp) {
//                    gamepad1.rumble(750);
//                    gamepad2.rumble(750);
//                    gamepad1.runRumbleEffect(whiteRumble);
//                    gamepad2.runRumbleEffect(whiteRumble);
//                    switch (getColor()) {
//                        case YELLOW:
//                            gamepad1.runRumbleEffect(yellowRumble);
//                            gamepad2.runRumbleEffect(yellowRumble);
//                            break;
//                        case GREEN:
//                            gamepad1.runRumbleEffect(greenRumble);
//                            gamepad2.runRumbleEffect(greenRumble);
//                            break;
//                        case PURPLE:
//                            gamepad1.runRumbleEffect(purpleRumble);
//                            gamepad2.runRumbleEffect(purpleRumble);
//                            break;
//                        case WHITE:
//                        default:
//                            gamepad1.runRumbleEffect(whiteRumble);
//                            gamepad2.runRumbleEffect(whiteRumble);
//                    }
                }
            }
            colorTriggered = true;
        } else {
            colorTriggered = false;
        }

        if(intakeScheduler != -1 && System.currentTimeMillis() > intakeScheduler && !teleOp) {
            if(teleOp) {
                intakeState = IntakeState.UNSTABLE_OUTPUT;
            } else {
                intakeState = IntakeState.OFF;
                frontStageOverride = false;
            }
            intakeScheduler = -1;
        }

        switch (intakeState) {
            case INPUT:
                intake.setPower(1);
                if(!frontStageOverride) frontStage.setPosition(0.76);
                break;
            case UNSTABLE_OUTPUT:
                if(!frontStageOverride) frontStage.setPosition(0.7);
                if(outputTimestamp == -1) {
                    outputTimestamp = System.currentTimeMillis();
                } else if(System.currentTimeMillis() - outputTimestamp > 1000) {
                    intakeState = IntakeState.OFF;
                    outputTimestamp = -1;
                }
                //Note the lack of a break clause here
            case OUTPUT:
                intake.setPower(-1);
                break;
            case OFF:
                if(!frontStageOverride) frontStage.setPosition(0.7);
                intake.setPower(0);
                break;
        }

        //Lifts
        switch (liftState) {
            case BASE:
                if(fingerOverride) {
                    setFingers(CLOSED, CLOSED);
                } else {
                    setFingers(OPEN, OPEN);
                }
                break;
            case AUTO_RAISE:
                leftLift.setPower(1);
                setFingers(CLOSED, CLOSED);
                fingerOverride = false;
                if(leftLift.getCurrentPosition() > minFlipHeight) {
                    liftState = LiftStates.FLIP_OUT;
                    leftLift.setPower(0);
                }
                break;
            case FLIP_OUT:
                setThroughPosition(ThruPositions.BACKDROP);
                if (flipTimestamp == -1) {
                    flipTimestamp = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - flipTimestamp > 500) {
                    liftState = LiftStates.VERTICAL_MANUAL;
                    flipTimestamp = -1;
                }
                break;
            case VERTICAL_MANUAL:
                setRotationPosition(RotationPositions.VERTICAL);
                FingerPositions left = leftDrop ? OPEN : CLOSED;
                FingerPositions right = rightDrop ? OPEN : CLOSED;
                setFingers(left, right);
                if(teleOp) {
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if(liftPower == 0) liftPower = 0.1;
                    leftLift.setPower(liftPower);
                } else {
                    if(liftSetpoint == 0) {
                        liftSetpoint = leftLift.getCurrentPosition();
                    }
                    leftLift.setPower(getPIDOutput(liftSetpoint));
                }
                break;
            case HORIZONTAL_MANUAL:
                setRotationPosition(RotationPositions.HORIZONTAL);
                FingerPositions left2 = leftDrop ? OPEN : CLOSED;
                FingerPositions right2 = rightDrop ? OPEN : CLOSED;
                setFingers(left2, right2);
                if(teleOp) {
                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if(liftPower == 0) liftPower = 0.1;
                    leftLift.setPower(liftPower);
                }
                break;
            case MIN_RAISE:
                rightDrop = false;
                leftDrop = false;
                liftSetpoint = 0;
                pixelCount = 0;
                setRotationPosition(RotationPositions.VERTICAL);
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(flipTimestamp == -1) {
                    flipTimestamp = System.currentTimeMillis();
                } else if(System.currentTimeMillis() - flipTimestamp > 500) {
                    flipTimestamp = -2;
                }
                if(leftLift.getCurrentPosition() > minFlipHeight) {
                    leftLift.setPower(0);
                    if(flipTimestamp == -2) {
                        liftState = LiftStates.FLIP_IN;
                        flipTimestamp = -1;
                    }
                } else {
                    leftLift.setPower(-1);
                }
                break;
            case FLIP_IN:
                setThroughPosition(ThruPositions.BASE);
                leftLift.setPower(0.1);
                if (flipTimestamp == -1) {
                    flipTimestamp = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - flipTimestamp > 500) {
                    liftState = LiftStates.BASE_RETURN;
                    flipTimestamp = -1;
                }
                break;
            case BASE_RETURN:
                leftLift.setPower(-0.5);

                if(gamepad2 != null) {
                    if (!liftTouch.getState() || gamepad2.start) {
                        leftLift.setPower(0);
                        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftState = LiftStates.BASE;
                    }
                } else {
                    if (!liftTouch.getState()) {
                        leftLift.setPower(0);
                        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftState = LiftStates.BASE;
                    }
                }

//                if (leftLift.getCurrentPosition() < 0) {
//                    leftLift.setPower(0);
//                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftState = LiftStates.BASE;
//                    fingerOverride = false;
//                }
                break;
            case FULL_MANUAL:
                if(liftSetpoint == 0) {
                    liftSetpoint = leftLift.getCurrentPosition();
                }
                leftLift.setPower(getPIDOutput(liftSetpoint));
                break;
            case PD_RAISE:
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftLift.setPower(1);
                if (leftLift.getCurrentPosition() > minFlipHeight) {
                    leftLift.setPower(0);
                    liftState = LiftStates.PD_FLIP;
                }
                break;
            case PD_FLIP:
                setThroughPosition(ThruPositions.PD);
                if (flipTimestamp == -1) {
                    flipTimestamp = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - flipTimestamp > 500) {
                    liftState = LiftStates.PD_LOWER;
                    flipTimestamp = -1;
                }
                break;
            case PD_LOWER:
                int set = 330;
                leftLift.setPower(getPIDOutput(set));
                //If error < 3, set to PD_READY
                if (Math.abs(leftLift.getCurrentPosition() - set) < 30) {
                    leftLift.setPower(0);
                    liftState = LiftStates.PD_READY;
                }
                break;
            case PD_READY:
                leftLift.setPower(0);
                break;
        }

        rightLift.setPower(leftLift.getPower());
    }

    //Public control methods
    //All methods return true if they have an effect or false if they fail
    public void toggleIntake() {
        frontStageOverride = false;
        switch (intakeState) {
            case INPUT:
                intakeState = IntakeState.UNSTABLE_OUTPUT;
                break;
            case OFF:
                intakeState = IntakeState.INPUT;
                break;
            case OUTPUT:
            case UNSTABLE_OUTPUT:
                intakeState = IntakeState.OFF;
                break;
        }
    }

    public void output() {
        frontStageOverride = false;
        intakeState = IntakeState.OUTPUT;
    }

    /** @noinspection unused*/
    public void frontStageOverride(int stackHeight) {
        frontStageOverride = true;
        //Ground 0.76
        //Up 0.7
        //5 Stack 0.735
        //4 Stack 0.74
        //3 Stack 0.75
        //2 Stack 0.755
        switch (stackHeight) {
            case 0:
                frontStage.setPosition(0.7);
                break;
            case 1:
                frontStage.setPosition(0.76);
                break;
            case 2:
                frontStage.setPosition(0.755);
                break;
            case 3:
                frontStage.setPosition(0.75);
                break;
            case 4:
                frontStage.setPosition(0.74);
                break;
            case 5:
                frontStage.setPosition(0.735);
                break;
        }
    }
    public boolean liftControl(double power){
        if(power == 0) {
            liftPower = 0;
            return false;
        }

        if(liftState == LiftStates.VERTICAL_MANUAL || liftState == LiftStates.HORIZONTAL_MANUAL) {
            liftPower = power;
            return true;
        }
        if (liftState == LiftStates.BASE && power > 0) {
            liftPower = 0;
            liftState = LiftStates.AUTO_RAISE;
            return true;
        }
        return false;
    }

    public void fingerOverrideBase(boolean yes) {
        if(yes && liftState == LiftStates.BASE) {
            fingerOverride = !fingerOverride;
        }
    }

    public void setFingerOverride(boolean b) {
        fingerOverride = b;
    }

    /** @noinspection unused*/
    public boolean liftSet(int position) {
        liftSetpoint = position;
        if(liftState == LiftStates.BASE) {
            liftState = LiftStates.AUTO_RAISE;
        }
        return true;
    }

    public boolean dropLeft(boolean drop) {
        if(!drop) {
            return false;
        }
        if(liftState == LiftStates.VERTICAL_MANUAL || liftState == LiftStates.HORIZONTAL_MANUAL) {
            leftDrop = true;
            return true;
        }
        return false;
    }

    public boolean dropRight(boolean drop) {
        if(!drop) {
            return false;
        }
        if(liftState == LiftStates.HORIZONTAL_MANUAL || liftState == LiftStates.VERTICAL_MANUAL) {
            rightDrop = true;
            if(liftState == LiftStates.VERTICAL_MANUAL) {
                leftDrop = true;
            }
            return true;
        }
        return false;
    }

    public boolean returnArm(boolean returnArm) {
        if (!returnArm) {
            return false;
        }
        if (liftState == LiftStates.VERTICAL_MANUAL || liftState == LiftStates.HORIZONTAL_MANUAL || liftState == LiftStates.PD_READY) {
            liftState = LiftStates.MIN_RAISE;
            return true;
        }
        return false;
    }

    public boolean flipHorizontal(boolean flip) {
        if (!flip) {
            return false;
        }
        if (liftState == LiftStates.VERTICAL_MANUAL) {
            liftState = LiftStates.HORIZONTAL_MANUAL;
            return true;
        }
        return false;
    }

    public boolean flipVertical(boolean flip) {
        if (!flip) {
            return false;
        }
        if (liftState == LiftStates.HORIZONTAL_MANUAL) {
            liftState = LiftStates.VERTICAL_MANUAL;
            return true;
        }
        return false;
    }

    public void enableFullManual() {
        liftState = LiftStates.FULL_MANUAL;
        liftSetpoint = 0;
    }

    /**
     * Disables full manual control. <b>The Lift will automatically rezero</b>
     */
    public void disableFullManual() {
        liftState = LiftStates.MIN_RAISE;
    }

    public void requestPD() {
        liftState = LiftStates.PD_RAISE;
    }

    public boolean isPDReady() {
        return liftState == LiftStates.PD_READY;
    }

    public void endPDresumeND() {
        liftState = LiftStates.AUTO_RAISE;
    }

    public boolean rumbleProcess() {
        boolean temp = rumbleRequested;
        rumbleRequested = false;
        return temp;
    }

    //Internal Private Methods
    private ColorDetectionSubsystem.BayColor getColor() {
        float[] HSV = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), HSV);

        Scalar colorScalar = new Scalar(HSV[0], HSV[1], HSV[2]);
        ColorDetectionSubsystem.BayColor colorOutput = WHITE;
        for (ColorDetectionSubsystem.BayColor color : ColorDetectionSubsystem.BayColor.values()) {
            if (isBigger(colorScalar, color.lower) && isSmaller(colorScalar, color.upper)) {
                colorOutput = color;
                break;
            }
        }

        return colorOutput;
    }

    /**
     * Returns true if all attributes of a are greater than b
     */
    private boolean isBigger(Scalar a, Scalar b) {
        return a.val[0] >= b.val[0] && a.val[1] >= b.val[1] && a.val[2] >= b.val[2];
    }

    /**
     * Returns true if all attributes of a are less than b
     */
    private boolean isSmaller(Scalar a, Scalar b) {
        return a.val[0] <= b.val[0] && a.val[1] <= b.val[1] && a.val[2] <= b.val[2];
    }

    public void setFingers(FingerPositions left, FingerPositions right) {
        if(left == OPEN) {
            leftFinger.setPosition(LEFT_OPEN.getPosition());
        } else if(left == CLOSED) {
            leftFinger.setPosition(LEFT_CLOSED.getPosition());
        } else {
            leftFinger.setPosition(left.getPosition());
        }

        if(right == OPEN) {
            rightFinger.setPosition(RIGHT_OPEN.getPosition());
        } else if(right == CLOSED) {
            rightFinger.setPosition(RIGHT_CLOSED.getPosition());
        } else {
            rightFinger.setPosition(right.getPosition());
        }
    }

    private void setThroughPosition(ThruPositions position) {
        thru.setPosition(position.getPosition());
    }
    private void setRotationPosition(RotationPositions position) {
        rotation.setPosition(position.getPosition());
    }

    private double getPIDOutput(int setpoint) {
        if(Math.abs(setpoint - leftLift.getCurrentPosition()) < 30) return 0.1;
        return armPID.calculate(leftLift.getCurrentPosition(), setpoint);
    }
}