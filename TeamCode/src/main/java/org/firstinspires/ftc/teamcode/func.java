package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class is used for global functions for easy cross program adjustments
 */
public class func /*extends LinearOpMode*/ {
    public static HardwareInit robot = new HardwareInit();

    public void setLEDss(boolean red, boolean green) {
        robot.leftLEDGreen.setState(green);
        robot.leftLEDRed.setState(red);
        robot.rightLEDGreen.setState(green);
        robot.rightLEDRed.setState(red);
    }
    public static void tankOn(double left, double right) {
        robot.leftBack.setPower(left);
        robot.leftFront.setPower(left);
        robot.rightBack.setPower(right);
        robot.rightFront.setPower(right);
    }
}
