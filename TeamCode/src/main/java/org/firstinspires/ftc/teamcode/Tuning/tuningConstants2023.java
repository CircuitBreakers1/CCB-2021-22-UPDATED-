package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;

@Config
public class tuningConstants2023 {

    public static double LEFTPOS = 0;
    public static double RIGHTPOS = 1;
    public static double GRIPOPEN = 0.47;
    public static double GRIPCLOSED = 0.36;

    public static double shootAngle = 0.5;
    public static double LEFTMULT = -1;
    public static double RIGHTMULT = -1;
    public static double BACKMULT = 1;
    public static double OFFSETMULT = -1;

    //Values are: kP, kI, kD, kF
    public static double NEWPIDFP = 0.17;
    public static double NEWPIDFI = 0.500736377;
    public static double NEWPIDFD = 0.0380919;

    public static int ARMBASE = -160;
    public static double ARMPICKUPANGLE = -3;
    public static double ARMROTATECONST = 0.0;

    public static double TurnPIDP = 1;
    public static double TurnPIDI = 0.0;
    public static double TurnPIDD = 0.0;

    public static double ARMP = 0.001;
    public static double ARMI = 0;
    public static double ARMD = 0;

    public static double startX = 0;
    public static double startY = 0;
    public static double startThetaPI = 0;

    public static boolean odoUseIMU = false;

    public static double endX = 0;
    public static double endY = 0;
    public static double endThetaPI = 0.25;

    public static double intakePower = 0.5;
}
