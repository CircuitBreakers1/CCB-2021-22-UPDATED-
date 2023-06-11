package org.firstinspires.ftc.teamcode.archive.PowerPlay.Auto;

import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.moveTo;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.turn;
import static org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems.PositionalMovementSubsystem.turnTo180;

/**
 * This class provides the framework to allow opposite alliance autos to be created in one program
 * and simply switch certain values depending on if the opmode is run from blue or red side.
 */
public class AutoSwitcher {
    static double x;
    static double y;
    static double heading;
    static autoStartSpot startSpot;

     static double mirror(double input, double reflectionPoint) {
         if(startSpot == autoStartSpot.LEFT) {
             return input;
         }
         return reflectionPoint - (input - reflectionPoint);
     }

     static double mirrorX(double input) {
        return mirror(input, x);
     }

     static double mirrorY(double input) {
         return mirror(input, y);
     }

     static double mirrorHeading(double input) {
         return mirror(input, heading);
     }

     static void setMirror(double xIn, double yIn, double theta) {
         x = xIn;
         y = yIn;
         heading = theta;
     }

     static void moveToMirrored(double endX, double endY, double speed, boolean exact) {
         moveTo(mirrorX(endX), endY, speed, exact);
     }

     static void moveToMirrored(double endX, double endY, double speed) {
         moveToMirrored(endX, endY, speed, true);
     }

     static void turnMirrored(double endHeading, double speed) {
         double mirroredHeading = mirrorHeading(endHeading);
         if(mirroredHeading == 180) {
             turnTo180(speed);
         } else {
             turn(mirrorHeading(endHeading), speed);
         }
     }


    /**
     * Setting non-null values will enable dynamic mirroring, which will mirror the values if the
     * right side is selected. Setting this to null will always mirror the values.
     * @param startSpot
     */
    static void setStartSpot(autoStartSpot startSpot) {
        AutoSwitcher.startSpot = startSpot;
    }
}


