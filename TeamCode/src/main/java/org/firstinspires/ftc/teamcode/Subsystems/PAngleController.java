package org.firstinspires.ftc.teamcode.Subsystems;

public class PAngleController {
    private double kP;

    public PAngleController(double kP) {
        this.kP = kP;
    }

    public double calculateOutput(double reference, double state) {
//        while(reference < 0) {
//            reference += 2 * Math.PI;
//        }
//        while(state < 0) {
//            state += 2 * Math.PI;
//        }
        double error = angleWrap(reference - state);

        return error * kP;
    }

    public double getError(double reference, double state) {
//        while(reference < 0) {
//            reference += 2 * Math.PI;
//        }
//        while(state < 0) {
//            state += 2 * Math.PI;
//        }
        return angleWrap(reference - state);
    }

    private double angleWrap(double rad) {
        while(rad > Math.PI) {
            rad -= Math.PI * 2;
        }
        while(rad < -Math.PI) {
            rad += Math.PI * 2;
        }
        return rad;
    }
}
