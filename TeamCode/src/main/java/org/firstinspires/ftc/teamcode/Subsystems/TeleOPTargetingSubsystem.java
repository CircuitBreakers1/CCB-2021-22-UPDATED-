package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.toDegrees;

import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

public class TeleOPTargetingSubsystem {
    private static HolonomicOdometry odometry;

    private static final double maxDistance = 1.0;
    private static final double maxAngleDeviation = 3.0;
    //Targeted junction on the 5x5 grid -- Each junction is 24 inches apart
    public static int[] targetJunctions = {1,1};
    public static double[] targetPosition = {0,0};
    public static double distance = 0.0;

    public static void initTargetingSubsystem(HolonomicOdometry inOdometry) {
        odometry = inOdometry;
    }

    public static boolean canDropCone() {
        //Determine which junction is closest. Track X axis then Y axis
        double distanceBetweenJunctions = 24;
        double x = odometry.getPose().getX(), y = odometry.getPose().getY();
        //Determine X axis
        for (int i = 1; i <= 5; i++) {
            if(i == 5) {
                targetJunctions[0] = i;
                break;
            }
            if(i * distanceBetweenJunctions + (distanceBetweenJunctions / 2) > x) {
                targetJunctions[0] = i;
                break;
            }
        }
        //Determines Y axis
        for (int i = 1; i <= 5; i++) {
            if(i == 5) {
                targetJunctions[1] = i;
                break;
            }
            if(i * distanceBetweenJunctions + (distanceBetweenJunctions / 2) > y) {
                targetJunctions[1] = i;
                break;
            }
        }

        targetPosition = new double[] {targetJunctions[0] * distanceBetweenJunctions, targetJunctions[1] * distanceBetweenJunctions};
        distance = Math.sqrt(Math.pow(targetPosition[0] - x, 2) + Math.pow(targetPosition[1] - y, 2));
        double angleToTarget = toDegrees(Math.atan2(targetPosition[1] - y, targetPosition[0] - x));

        return !(distance > maxDistance) && !(angleToTarget > maxAngleDeviation);
    }

}
