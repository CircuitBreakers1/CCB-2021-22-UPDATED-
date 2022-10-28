package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.VisionPipeline.SignalColor.*;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {

    public enum SignalColor {
        RED, GREEN, BLUE
    }

    private static final double rectWidthPercentage = 0.65, rectHeightPercentage = 0.5;
    private static final int rectangleWidth = 70, rectangleHeight = 100;
    private double average;
    private Mat matAdapted = new Mat(), block = new Mat();
    private Mat matHue = new Mat();
    private SignalColor predictedColor = RED;

    @Override
    public Mat processFrame(Mat input) {


        //First, create the Mat used to find the hue of the box
        Imgproc.cvtColor(input, matAdapted, Imgproc.COLOR_RGB2HSV);

        Rect rect = new Rect(
                (int) (matAdapted.width() * rectWidthPercentage),
                (int) (matAdapted.height() * rectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        block = matAdapted.submat(rect);

        Core.extractChannel(block, matHue, 0);

        Scalar mean = Core.mean(matHue);

        average = mean.val[0];


        //This if uses what hopefully comes out to the HSV value that is needed to check the color
        if(average <= 60/2 || (average >= 300/2 && average <= 360/2)) {
            predictedColor = RED;
        } else if (average > 60/2 && average <= 180/2) {
            predictedColor = GREEN;
        } else if (average > 180/2 && average < 300/2) {
            predictedColor = BLUE;
        }


        //Then create the Mat used for the viewport
        //"Evan Proof Viewing"
        Scalar color = new Scalar(0,0,0,0);

        Imgproc.rectangle(input, rect, color, 7);

        return input;
    }


    public SignalColor getPredictedColor() {
        return predictedColor;
    }




    /**
     * For debug purposes only
     * @return Average Hue value of target square
     */
    public double getAverage() {
        return average;
    }
}


