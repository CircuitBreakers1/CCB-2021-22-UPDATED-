package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class tuningConstants {

    //Values are: kP, kI, kD, kF
    public static double FORWARDPIDFP = 0.5;
    public static double FORWARDPIDFI = 0;
    public static double FORWARDPIDFD = 0;
    public static double FORWARDPIDFF = 0;
    public static double ANGLEPIDFP = 0;
    public static double ANGLEPIDFI = 0;
    public static double ANGLEPIDFD = 0;
    public static double ANGLEPIDFF = 0;

    public static double trackTuning = 12;

}
