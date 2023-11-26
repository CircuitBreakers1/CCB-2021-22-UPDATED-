package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ColorDetectionSubsystem.BayColor.EMPTY;
import static org.firstinspires.ftc.teamcode.Subsystems.ColorDetectionSubsystem.BayColor.UNKNOWN;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import javax.annotation.Nullable;

public class ColorDetectionSubsystem {
    private NormalizedColorSensor leftBay;
    private NormalizedColorSensor rightBay;

    float gain = 2;
    double maxDist = 2;

    //TODO: Tune Colors

    /**
     * The potential pixel colors stored in the bays. UNKNOWN is used when the distance sensor determines there is a pixel,
     * but the color is not recognized
     */
    public enum BayColor {
        PURPLE(new Scalar(217, 0.65, 0.15), new Scalar(223, 0.7, 0.5)), YELLOW(new Scalar(91, 0.65, 0), new Scalar(96, 1, 1)),
        GREEN(new Scalar(127, 0.7, 0), new Scalar(139, 1, 1)), WHITE(new Scalar(188, 0, 0), new Scalar(195, 1, 1)),
        UNKNOWN(new Scalar(0), new Scalar(0)), EMPTY(new Scalar(0), new Scalar(0));
        final Scalar lower;
        final Scalar upper;

        BayColor(Scalar lower, Scalar upper) {
            //First one is hue in degrees, then % saturation, then % value, so we scale it
            this.upper = new Scalar(upper.val[0], upper.val[1], upper.val[2]);
            this.lower = new Scalar(lower.val[0], lower.val[1], lower.val[2]);
        }
    }

    public ColorDetectionSubsystem(NormalizedColorSensor left, NormalizedColorSensor right) {
        leftBay = left;
        rightBay = right;
    }

    /**
     * Get the color of the bays
     * @return The color of the bays, stored in an Array in order [left, right]. Null if no color is detected
     */
    public BayColor[] getBayColors() {
        BayColor[] colors = {EMPTY, EMPTY};
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.colorToHSV(leftBay.getNormalizedColors().toColor(), leftHSV);
        Color.colorToHSV(rightBay.getNormalizedColors().toColor(), rightHSV);
        Scalar left = new Scalar(leftHSV[0], leftHSV[1], leftHSV[2]);
        Scalar right = new Scalar(rightHSV[0], rightHSV[1], rightHSV[2]);
        //Run the loop for each bay
        if(((DistanceSensor) leftBay).getDistance(DistanceUnit.CM) < maxDist){
            colors[0] = UNKNOWN;
            for (BayColor color: BayColor.values()) {
                if(isBigger(left, color.lower) && isSmaller(left, color.upper)) {
                    colors[0] = color;
                    break;
                }
            }
        }
        if(((DistanceSensor) rightBay).getDistance(DistanceUnit.CM) < maxDist){
            colors[1] = UNKNOWN;
            for (BayColor color : BayColor.values()) {
                if (isBigger(right, color.lower) && isSmaller(right, color.upper)) {
                    colors[1] = color;
                    break;
                }
            }
        }
        return colors;
    }

    /**
     * Returns true if all attributes of a are greater than b
     */
    private boolean isBigger(Scalar a, Scalar b) {
        return a.val[0] >= b.val[0] && a.val[1] >= b.val[1] && a.val[2] >= b.val[2];
    }

    /**
     * Returns true if all attributes of a are less than b
     */
    private boolean isSmaller(Scalar a, Scalar b) {
        return a.val[0] <= b.val[0] && a.val[1] <= b.val[1] && a.val[2] <= b.val[2];
    }

    /**
     * For debug purposes. Returns the HSV values of the left bay or -1 if no pixel is detected
     */
    public float[] getLeftHSV() {
        if(((DistanceSensor) leftBay).getDistance(DistanceUnit.CM) > maxDist) return new float[]{-1.0f, -1.0f, -1.0f};
        float[] hsv = new float[3];
        Color.colorToHSV(leftBay.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    /**
     * For debug purposes. Returns the HSV values of the right bay, or -1 if no pixel is detected
     */
    public float[] getRightHSV() {
        if(((DistanceSensor) rightBay).getDistance(DistanceUnit.CM) > maxDist) return new float[]{-1.0f, -1.0f, -1.0f};
        float[] hsv = new float[3];
        Color.colorToHSV(rightBay.getNormalizedColors().toColor(), hsv);
        return hsv;
    }
}
