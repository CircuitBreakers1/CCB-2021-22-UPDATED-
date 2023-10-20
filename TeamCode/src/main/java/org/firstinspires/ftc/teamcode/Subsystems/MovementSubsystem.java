package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Robot2023.LOGCATTAG;
import static java.lang.Math.abs;
import static java.lang.Math.exp;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.util.RobotLog;

public class MovementSubsystem {
    private final HoloDrivetrainSubsystem holoDrivetrain;
    private final HolonomicOdometry holOdom;

    private static final double precision = 0.5;

    private final double V1 = 0.1, V2 = 0.999;
    /**
     * Distance from point to begin slowing down
     */
    private final double D1 = 0.0, D2 = 6.0;
    private final double A = V1 / (1 - V1);
    private final double K = (1 / D2) * Math.log(V2 / (A * (1 / V2)));

    public MovementSubsystem(HoloDrivetrainSubsystem holoDrivetrain, HolonomicOdometry holOdom) {
        this.holoDrivetrain = holoDrivetrain;
        this.holOdom = holOdom;
    }

    public void moveTo(double x, double y, double theta, double maxSpeed) {
        RobotLog.d(LOGCATTAG + "Moving to " + x + ", " + y + ", " + theta);
        double distance = Math.hypot(x - holOdom.getPose().getX(), y - holOdom.getPose().getY());
        RobotLog.v(LOGCATTAG + "Distance: " + distance);

        double xSpeed, ySpeed, turnSpeed, xDistance, yDistance, turnDistance, heading;
        Pose2d pose;
        while(distance > precision) {
            holOdom.updatePose();
            pose = holOdom.getPose();
            xDistance = x - pose.getX();
            yDistance = y - pose.getY();
            heading = pose.getHeading();
            turnDistance = theta - heading;
            distance = Math.hypot(xDistance, yDistance);
            xSpeed = getSpeedFromDistance(xDistance);
            ySpeed = getSpeedFromDistance(yDistance);

            if(xSpeed > maxSpeed) {
                xSpeed = maxSpeed;
            }
            if(ySpeed > maxSpeed) {
                ySpeed = maxSpeed;
            }
            if(ySpeed < 0.001) {
                ySpeed = 0;
            }
            if(xSpeed < 0.001) {
                xSpeed = 0;
            }

            double x_rotated = xSpeed * Math.cos(heading) - ySpeed * Math.sin(heading);
            double y_rotated = xSpeed * Math.sin(heading) + ySpeed * Math.cos(heading);

            double lfSpeed = x_rotated - y_rotated;
            double lbSpeed = x_rotated + y_rotated;
            double rfSpeed = x_rotated + y_rotated;
            double rbSpeed = x_rotated - y_rotated;

            double lfTemp = abs(lfSpeed);
            double lbTemp = abs(lbSpeed);
            double rfTemp = abs(rfSpeed);
            double rbTemp = abs(rbSpeed);

            double maxMotorSpeed = max(lfTemp, max(lbTemp, max(rfTemp, rbTemp)));

            if(maxMotorSpeed > 1) {
                lfSpeed /= maxMotorSpeed;
                lbSpeed /= maxMotorSpeed;
                rfSpeed /= maxMotorSpeed;
                rbSpeed /= maxMotorSpeed;
            }

            holoDrivetrain.drive(lfSpeed, rfSpeed, lbSpeed, rbSpeed);

            RobotLog.v(LOGCATTAG + "X Speed: " + xSpeed + ", Y Speed: " + ySpeed + ", X Distance: " + xDistance + ", Y Distance: " + yDistance + ", Heading: " + heading + ", Turn Distance: " + turnDistance + ", Distance: " + distance);
        }
        RobotLog.d(LOGCATTAG + "Finished moving to " + x + ", " + y + ", " + theta + ". Actual Position: " + holOdom.getPose().toString());
        holoDrivetrain.stop();
    }

    private double getSpeedFromDistance(double distance) {
        if(abs(distance) < precision) {
            return 0;
        }
        int sign = distance < 0 ? -1 : 1;

        distance = abs(distance);

        double top = A * exp(K * distance);
        double bottom = 1 + (top);

        return sign * top / bottom;
    }
}
