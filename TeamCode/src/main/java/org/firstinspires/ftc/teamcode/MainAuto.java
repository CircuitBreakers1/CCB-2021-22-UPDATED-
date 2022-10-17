package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MainAuto implements AutoSwitcher {
    autoStartSpot startSpot = null;
    OpMode opMode = null;

    public MainAuto(autoStartSpot givenStart, OpMode givenOpMode) {
        startSpot = givenStart;
        opMode = givenOpMode;
    }

    public void initAuto() {

    }

    public void runAuto() {

    }
}

enum autoStartSpot {
    RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT
}