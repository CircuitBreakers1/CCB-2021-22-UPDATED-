package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.opencv.core.Scalar;

import javax.annotation.Nullable;

public class ColorDetectionSubsystem {
    private NormalizedColorSensor leftBay;
    private NormalizedColorSensor rightBay;

    float gain = 2;

    //TODO: Tune Colors
    @Nullable
    public enum BayColor {
        PURPLE(new Scalar(0), new Scalar(0)), YELLOW(new Scalar(0), new Scalar(0)), GREEN(new Scalar(0), new Scalar(0)), WHITE(new Scalar(0), new Scalar(0));
        final Scalar lower;
        final Scalar upper;

        BayColor(Scalar upper, Scalar lower) {
            this.upper = upper;
            this.lower = lower;
        }
    }

    public ColorDetectionSubsystem(NormalizedColorSensor left, NormalizedColorSensor right) {
        leftBay = left;
        rightBay = right;
    }

    public BayColor[] getBayColors() {
        float[] leftRGB = {leftBay.getNormalizedColors().red * 255,
                leftBay.getNormalizedColors().green * 255,
                leftBay.getNormalizedColors().blue * 255};
        float[] rightRGB = {rightBay.getNormalizedColors().red * 255,
                rightBay.getNormalizedColors().green * 255,
                rightBay.getNormalizedColors().blue * 255};
        for (BayColor color: BayColor.values()) {

        }
        return null;
    }
}
