package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class robot {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public robot(){

    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void robotPath() {

    }
}
