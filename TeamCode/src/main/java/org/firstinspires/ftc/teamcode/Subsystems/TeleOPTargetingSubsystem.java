package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

public class TeleOPTargetingSubsystem {
    private static HolonomicOdometry odometry;

    private static final double maxDistance = 1.0;
    private static final double maxAngleDeviation = 0.0;
    //Targeted junction on the 5x5 grid -- Each junction is 24 inches apart
    public static int[] targetJunctions = {1,1};
    public static double[] targetPosition = {0,0};
    public static double distance = 0.0;

    public static void initTargetingSubsystem(HolonomicOdometry inOdometry) {
        odometry = inOdometry;
    }

    public static boolean canDropCone() {
        double x = odometry.getPose().getX(), y = odometry.getPose().getY();

        targetJunctions = new int[]{1, 1};

        for(; targetJunctions[0] > 5; targetJunctions[0]++) {
            double xTarget = targetJunctions[0] * 24;
            if(xTarget - maxDistance < x && x < xTarget + maxDistance) {
                for(; targetJunctions[1] > 5; targetJunctions[1]++) {
                    double yTarget = targetJunctions[1] * 24;
                    if(yTarget - maxDistance < y && y < yTarget + maxDistance) {
                        break;
                    }
                    if(yTarget - maxDistance > y) {
                        return false;
                    }
                }
            }
            //If we go past our location on the x axis, we can't drop a cone so break
            if(xTarget - maxDistance > x) {
                return false;
            }
        }



        //Make sure that we are within the max angle deviation
        double angle = odometry.getPose().getHeading();
        double deltaX = targetPosition[0] - x, deltaY = targetPosition[1] - y;
        double targetAngle = Math.atan2(deltaY, deltaX);
        distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        if(targetAngle - maxAngleDeviation < angle && angle < targetAngle + maxAngleDeviation && distance <= maxDistance) {
            return true;
        }
        return false;
    }

}
