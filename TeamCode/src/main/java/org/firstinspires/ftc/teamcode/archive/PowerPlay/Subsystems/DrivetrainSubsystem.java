package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DrivetrainSubsystem {
    private final DcMotorEx lf, rf, lb, rb;
    public static MecanumDrive mecDrive;
    private boolean useTuner = false, followMotionRules = false;


    private final double[] tuneConstants;
    private final double[] preloadedTune = {1.0,1.0,1.0,1.0};
                                    //Max V, Max A, Max J
                                    //in/s, in/s^2, in/s^3
    private final double[] motionRules = {1.0, 1.0, 1.0};

    /**
     * Initalize the Drivetrain
     * @param leftf Left Front Motor Reference
     * @param rightf Right Front Motor Reference
     * @param leftb Left Back Motor Reference
     * @param rightb Right Back Motor Reference
     */
    public DrivetrainSubsystem(DcMotorEx leftf, DcMotorEx rightf, DcMotorEx leftb, DcMotorEx rightb) {
        lf = leftf;
        rf = rightf;
        lb = leftb;
        rb = rightb;
        this.tuneConstants = new double[]{0, 0, 0, 0};

        useTuner = false;
    }

    public DrivetrainSubsystem(DcMotorEx leftf, DcMotorEx rightf, DcMotorEx leftb, DcMotorEx rightb, double[] tuneConstants) {
        lf = leftf;
        rf = rightf;
        lb = leftb;
        rb = rightb;

        this.tuneConstants = tuneConstants;

        useTuner = true;
    }

    /**
     * Move the drive motors at specified speeds
     * @param lfSpeed Left Front Speed
     * @param rfSpeed Right Front Speed
     * @param lbSpeed Left Back Speed
     * @param rbSpeed Right Back Speed
     */
    public void drive(double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed) {
        if(useTuner) {
            lf.setPower(lfSpeed * tuneConstants[0]);
            rf.setPower(rfSpeed * tuneConstants[1]);
            lb.setPower(lbSpeed * tuneConstants[2]);
            rb.setPower(rbSpeed * tuneConstants[3]);
        } else {
            lf.setPower(lfSpeed);
            rf.setPower(rfSpeed);
            lb.setPower(lbSpeed);
            rb.setPower(rbSpeed);
        }
    }

    /**
     * Move the drivetrain in a tank drive setup
     * @param leftSide Left Side Speed
     * @param rightSide Right Side Speed
     */
    public void drive(double leftSide, double rightSide) {
        drive(leftSide, rightSide, leftSide, rightSide);
    }

    public void drive(double speed) {
        drive(speed, speed, speed, speed);
    }

    /**
     * Drive holonomic (Robot relative)
     * @param forward Forward Input
     * @param strafe Strafe Input
     * @param rotation Rotation Input
     * @param speedMultipiler Max Speed
     */
    public void driveHolo(double forward, double strafe, double rotation, double speedMultipiler) {
        double lfSpeed = speedMultipiler * (forward + strafe + rotation);
        double rfSpeed = speedMultipiler * (forward - strafe - rotation);
        double lbSpeed = speedMultipiler * (forward - strafe + rotation);
        double rbSpeed = speedMultipiler * (forward + strafe - rotation);

        drive(lfSpeed, rfSpeed, lbSpeed, rbSpeed);
    }

    /**
     * Stops all drivetrain movement
     */
    public void stop() {
        drive(0,0);
    }

    public void setFollowMotionRules(boolean followMotionRules) {
        this.followMotionRules = followMotionRules;
    }

    public void resetEncoders() {
        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
