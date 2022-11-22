package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class tuningConstants {

    //Values are: kP, kI, kD, kF
    public static double FORWARDPIDFP = 0.7;
    public static double FORWARDPIDFI = 0.9;
    public static double FORWARDPIDFD = -0.2;
    public static double FORWARDPIDFF = 0;
    public static double HORIZONTALPIDFP = 0.7;
    public static double HORIZONTALPIDFI = 0.9;
    public static double HORIZONTALPIDFD = -0.2;
    public static double DistancePIDFP = 1;
    public static double DistancePIDFI = 0;
    public static double DistancePIDFD = 0;
//    public static double ANGLEPIDFP = 0;
//    public static double ANGLEPIDFI = 0;
//    public static double ANGLEPIDFD = 0;
//    public static double ANGLEPIDFF = 0;

    public static double trackTuning = 11.95;
}
