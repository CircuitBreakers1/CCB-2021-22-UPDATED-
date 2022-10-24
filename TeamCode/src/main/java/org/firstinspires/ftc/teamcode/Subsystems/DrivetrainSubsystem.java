package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DrivetrainSubsystem {
    private final MotorEx lf, rf, lb, rb;
    public static MecanumDrive mecDrive;

    /**
     * Initalize the Drivetrain
     * @param leftf Left Front Motor Reference
     * @param rightf Right Front Motor Reference
     * @param leftb Left Back Motor Reference
     * @param rightb Right Back Motor Reference
     */
    public DrivetrainSubsystem(MotorEx leftf, MotorEx rightf, MotorEx leftb, MotorEx rightb) {
        lf = leftf;
        rf = rightf;
        lb = leftb;
        rb = rightb;

        mecDrive = new MecanumDrive(false, lf, rf, lb, rb);
    }

    /**
     * Move the drive motors at specified speeds
     * @param lfSpeed Left Front Speed
     * @param rfSpeed Right Front Speed
     * @param lbSpeed Left Back Speed
     * @param rbSpeed Right Back Speed
     */
    public void drive(double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed) {
        lf.set(lfSpeed);
        rf.set(rfSpeed);
        lb.set(lbSpeed);
        rb.set(rbSpeed);
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
}
