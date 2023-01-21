package org.firstinspires.ftc.teamcode.Subsystems;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ContourStackPipeline extends OpenCvPipeline {

    private Mat gray = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private List<ContourEx> filteredContours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private double stackX;
    private double stackWidth;
    private double camPXWidth = 0;
    //The width of the camera perspective at the depth of the stack
    private final double cameraViewWidthIn = 12;

    @Override
    public Mat processFrame(Mat input) {
        if(camPXWidth == 0) {
            camPXWidth = input.width();
        }
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        //Find contours, and draw them on the screen
        //TODO: Tune thresh
        contours.clear();
        Imgproc.threshold(gray, gray, 150, 255, Imgproc.THRESH_BINARY);
        Imgproc.findContours(gray, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 3);

        //Filter contours for size
        filteredContours.clear();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            //TODO: Tune area
            if (area > 1000) {
                Imgproc.rectangle(input, Imgproc.boundingRect(contour), new Scalar(255, 0, 0), 3);
                Imgproc.putText(input, "Size: " + area, Imgproc.boundingRect(contour).tl(), 0, 1, new Scalar(0, 0, 0));
                filteredContours.add(new ContourEx(contour, area));
            } else {
                //Remove after tuning
                Imgproc.rectangle(input, Imgproc.boundingRect(contour), new Scalar(0, 0, 255), 3);
                Imgproc.putText(input, "Size: " + area, Imgproc.boundingRect(contour).tl(), 0, 1, new Scalar(0, 0, 0));
            }
        }

        //Determine which contour is the cone stack by lowest y value
        ContourEx lowestContour = null;
        if(filteredContours.size() > 1) {
            for (ContourEx contour : filteredContours) {
                if (lowestContour == null || contour.bottomY > lowestContour.bottomY) {
                    lowestContour = contour;
                }
            }
        } else if (filteredContours.size() == 1) {
            lowestContour = filteredContours.get(0);
        } else {
            lowestContour = null;
        }

        try {
            stackX = lowestContour.centerX;
            stackWidth = lowestContour.getContour().width();
        } catch (NullPointerException e) {
            stackX = -1;
            stackWidth = -1;
        }

        return input;
    }

    public double getStackOffset() {
        try {
            double cameraViewWidthHalf = cameraViewWidthIn / 2;
            return (stackX / camPXWidth) * cameraViewWidthIn - cameraViewWidthHalf;
        } catch (NullPointerException e) {
            return -1;
        }
    }

    public boolean canSeeStack() {
        return stackX != -1;
    }

    class ContourEx {
        private MatOfPoint contour;
        private double area;
        private double bottomY;
        private double centerX;

        //Use this to save processing time
        public ContourEx(MatOfPoint contour, double area) {
            this.contour = contour;
            this.area = area;
            this.bottomY = Imgproc.boundingRect(contour).br().y;
            this.centerX = Imgproc.boundingRect(contour).tl().x + (Imgproc.boundingRect(contour).width / 2d);
        }

        public MatOfPoint getContour() {
            return contour;
        }

        public double getArea() {
            return area;
        }
    }
}