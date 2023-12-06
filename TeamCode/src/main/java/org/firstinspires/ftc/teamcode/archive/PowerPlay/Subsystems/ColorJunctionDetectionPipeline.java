package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorJunctionDetectionPipeline extends OpenCvPipeline {


    private static double rectWidthPercentage, rectHeightPercentage;
    private static final int rectangleWidth = 30, rectangleHeight = 50;
    private Mat matAdapted = new Mat(), block = new Mat();
    private Mat matHue = new Mat();
    private final int subRects = 1;
    private boolean junctionDetected = false;
    private double[] average = new double[subRects];

    private final int outerRange = 5;

    public ColorJunctionDetectionPipeline(boolean leftCamera) {
        if (leftCamera) {
            rectWidthPercentage = 0.63;
            rectHeightPercentage = 0.61;
        } else {
            rectWidthPercentage = 0.27;
            rectHeightPercentage = 0.39;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        //First, create the Mat used to find the hue of the box
        Imgproc.cvtColor(input, matAdapted, Imgproc.COLOR_RGB2YCrCb);

        Rect rect = new Rect(
                (int) (matAdapted.width() * rectWidthPercentage),
                (int) (matAdapted.height() * rectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        Rect outerRect = new Rect(
                (int) (rect.tl().x - outerRange),
                (int) (rect.tl().y - outerRange),
                rectangleWidth + 2 * outerRange,
                rectangleHeight + 2 * outerRange
        );

        //Create two subrectangles to increase accuracy
        synchronized (this) {
            junctionDetected = false;
            for (int i = 0; i < subRects; i++) {
                Rect subRect = new Rect(
                        rect.x + (i * (rect.width / subRects)),
                        rect.y,
                        rect.width / subRects,
                        rect.height
                );

                block = matAdapted.submat(subRect);

                Core.extractChannel(block, matHue, 2);

                Scalar mean = Core.mean(matHue);

                average[i] = mean.val[0];

                if (average[i] > 160) {
                    junctionDetected = true;
                }
            }
            block = matAdapted.submat(outerRect);
            Core.extractChannel(block, matHue, 2);

            Scalar mean = Core.mean(matHue);

            average[0] = mean.val[0];
            if (average[0] > 160) {
                junctionDetected = false;
            }
        }

        //Then create the Mat used for the viewport
        //"Evan Proof Viewing"
        Scalar red = new Scalar(0, 0, 255, 0);
        Scalar green = new Scalar(0, 255, 0, 0);

        Imgproc.rectangle(input, rect, green, 2);
        Imgproc.rectangle(input, outerRect, red, 2);

        return input;
    }

    public boolean isJunctionDetected() {
        return junctionDetected;
    }

    /**
     * For debug purposes only
     *
     * @return Average Hue value of target square
     */
    public double[] getAverage() {
        return average;
    }
}


