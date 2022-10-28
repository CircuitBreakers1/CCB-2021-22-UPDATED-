package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.function.DoubleSupplier;

public class OdometrySubsystem {
    private DoubleSupplier leftOdo, rightOdo, backOdo;
    private double trackwidth, horizontalOffset;
    private double[] position;
    private double prevLeft, prevRight, prevBack;

    public OdometrySubsystem(DoubleSupplier left, DoubleSupplier right, DoubleSupplier back, double odoWidth, double backOffset, double startX, double startY, double startHeading) {
        this.leftOdo = left;
        this.rightOdo = right;
        this.backOdo = back;
        this.trackwidth = odoWidth;
        this.horizontalOffset = backOffset;
        position = new double[]{startX, startY, startHeading};
    }

    public OdometrySubsystem(DoubleSupplier left, DoubleSupplier right, DoubleSupplier back, double odoWidth, double backOffset) {
        this(left, right, back, odoWidth, backOffset, 0, 0, 0);
    }

    public double[] getPosition() {
        return position;
    }

    public double getX() {
        return position[0];
    }

    public double getY() {
        return position[1];
    }

    public double getHeading() {
        return position[2];
    }

    public void updatePosition() {
        double dLeft, dRight, dBack;
        double dX, dY, dTheta;

        double currLeft = leftOdo.getAsDouble();
        double currRight = rightOdo.getAsDouble();
        double currBack = backOdo.getAsDouble();

        dLeft = currLeft - prevLeft;
        dRight = currRight - prevRight;
        dBack = currBack - prevBack;




        prevLeft = currLeft;
        prevRight = currRight;
        prevBack = currBack;
    }
}

