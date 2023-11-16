package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class tuningConstants {


    public static double FORWARDPIDFP = 0.85;
    public static double FORWARDPIDFI = 1.62;
    public static double FORWARDPIDFD = 0.095;
    public static double FORWARDPIDFF = 0;

    public static double ANGLEPIDFP = 2;
    public static double ANGLEPIDFI = 0;
    public static double ANGLEPIDFD = 0;


    public static double trackTuning = 11.95;
}
