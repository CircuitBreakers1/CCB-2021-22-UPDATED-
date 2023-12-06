package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropGuess.LEFT;
import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropGuess.MIDDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropGuess.RIGHT;
import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropGuess.UNKNOWN;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_HUE_UPPER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_SAT_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_SAT_UPPER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_VAL_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.BLUE_VAL_UPPER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_HUE_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_HUE_UPPER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_SAT_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_SAT_UPPER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_VAL_LOWER;
import static org.firstinspires.ftc.teamcode.Tuning.ColorTuning.RED_VAL_UPPER;
import static org.opencv.imgproc.Imgproc.boundingRect;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ColorBlobDetector implements VisionProcessor {

    public enum PropGuess {
        LEFT, RIGHT, MIDDLE, UNKNOWN
    }

    public enum PropColor {
        RED, BLUE
    }

    PropGuess guess = UNKNOWN;
    PropColor color;

    double leftPercent = 0.33;
    double rightPercent = 0.33;

    public double sigma = 5;

    public Scalar lower;
    public Scalar upper;
    public Scalar lower2;
    public Scalar upper2;

    int gaussSize = 9;

    public int minSize = 2500;

    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();
    Mat processed;
    Mat thresh2;
    Mat thresh;
    Point mid;
    int midX;
    int midY;
    Size gSize;

    public ColorBlobDetector(PropColor propColor) {
        this.color = propColor;
        switch (propColor) {
            case RED:
                this.lower = new Scalar(0, RED_SAT_LOWER, RED_VAL_LOWER); //(0, 40, 40); // the lower hsv threshold for your detection
                this.upper = new Scalar(RED_HUE_UPPER, RED_SAT_UPPER, RED_VAL_UPPER); //(30, 255, 255); // the upper hsv threshold for your detection
                this.lower2 = new Scalar(RED_HUE_LOWER, RED_SAT_LOWER, RED_VAL_LOWER);
                this.upper2 = new Scalar(179, RED_SAT_UPPER, RED_VAL_UPPER);
                break;
            case BLUE:
                this.lower = new Scalar(BLUE_HUE_LOWER, BLUE_SAT_LOWER, BLUE_VAL_LOWER); //(90, 40, 40); // the lower hsv threshold for your detection
                this.upper = new Scalar(BLUE_HUE_UPPER, BLUE_SAT_UPPER, BLUE_VAL_UPPER); //(150, 255, 255); // the upper hsv threshold for your detection
                break;
        }
    }

    public PropGuess getGuess() {
        return guess;
    }

    /**
     * We have this method, but we don't need it and don't use it for this class
     *
     * @param width
     * @param height
     * @param calibration
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        contours.clear();

        processed = new Mat();
        gSize = new Size(gaussSize, gaussSize);
        Imgproc.GaussianBlur(input, processed, gSize, sigma);

        midX = input.width() / 2;
        midY = input.height() / 2;
        mid = new Point(midX, midY);

        thresh = new Mat();
        Imgproc.cvtColor(processed, processed, Imgproc.COLOR_RGB2HSV);

        Core.inRange(processed, lower, upper, thresh);
        if(color == PropColor.RED) {
            thresh2 = new Mat();
            Core.inRange(processed, lower2, upper2, thresh2);
            Core.bitwise_or(thresh, thresh2, thresh);
        }

        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.THRESH_BINARY_INV, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 255, 255), 1);


        //Draw bounding lines for tuning now that the contours are already determined
        Imgproc.line(input, new Point(input.width() * leftPercent,0), new Point(input.width() * leftPercent, input.height()), new Scalar(255,255,255));
        Imgproc.line(input, new Point(input.width() * (1 - rightPercent), 0), new Point(input.width() * (1 - rightPercent), input.height()), new Scalar(255, 255, 255));
        PropGuess currentGuess = UNKNOWN;


        for (MatOfPoint contour : contours) {
            Rect boundingRect = boundingRect(contour);
            if (boundingRect.size().area() < minSize) continue;
            Imgproc.rectangle(input, boundingRect, new Scalar(0, 0, 255), 2);
            Rect textBox = new Rect((int) boundingRect.tl().x - 1, (int) boundingRect.tl().y - 10, 40, 10);
            Imgproc.rectangle(input, textBox, new Scalar(0, 0, 255), -1);
            Imgproc.putText(input, String.valueOf((int) boundingRect.area()), new Point(textBox.tl().x, textBox.br().y - 2), 6, 0.35, new Scalar(0, 0, 0));
            int centerX = (int) (boundingRect.tl().x + boundingRect.br().x) / 2;
            if (centerX < input.width() * leftPercent) {
                currentGuess = LEFT;
            } else if (centerX > input.width() * (1 - rightPercent)) {
                currentGuess = RIGHT;
            } else {
                currentGuess = MIDDLE;
            }
        }
        if (currentGuess != guess && currentGuess != UNKNOWN) {
            guess = currentGuess;
        }

//        telemetry.addData("Contours Found", contours.size());
//        telemetry.addData("Prop Guess", guess);
//        telemetry.update();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (filteredContours.size() > 0) {
            Paint paint = new Paint();
            paint.setColor(0x0);
            for (MatOfPoint contour : filteredContours) {
                Rect rect = boundingRect(contour);
                canvas.drawRect(new android.graphics.Rect((int) (rect.tl().x * scaleBmpPxToCanvasPx), (int) (rect.tl().y * scaleBmpPxToCanvasPx), (int) (rect.br().x * scaleBmpPxToCanvasPx), (int) (rect.br().y * scaleBmpPxToCanvasPx)), paint);
            }
        }
    }
}
