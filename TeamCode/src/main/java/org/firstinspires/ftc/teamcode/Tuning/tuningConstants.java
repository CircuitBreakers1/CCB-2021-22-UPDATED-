package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class tuningConstants {

    //Values are: kP, kI, kD, kF
    public static double NEWPIDFP = 0.85;
    public static double NEWPIDFI = 0;
    public static double NEWPIDFD = 0;


    public static double FORWARDPIDFP = 0.85;
    public static double FORWARDPIDFI = 1.62;
    public static double FORWARDPIDFD = 0.095;
    public static double FORWARDPIDFF = 0;
    public static double HORIZONTALPIDFP = 0.85;
    public static double HORIZONTALPIDFI = 2;
    public static double HORIZONTALPIDFD = 0.13;
    public static double DistancePIDFP = 1;
    public static double DistancePIDFI = 0;
    public static double DistancePIDFD = 0;
    public static double ANGLEPIDFP = 2;
    public static double ANGLEPIDFI = 0;
    public static double ANGLEPIDFD = 0;
    public static double ANGLEPIDFF = 0;

    public static double trackTuning = 11.95;
}
