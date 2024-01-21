package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropColor.BLUE;
import static org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector.PropColor.RED;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import android.util.Size;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import javax.annotation.Nullable;

public class CameraSubsystem {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ColorBlobDetector.PropColor currentColor = null;
    private ColorBlobDetector blueBlobDetector;
    private ColorBlobDetector redBlobDetector;

    //X Y Displacement on robot from Center
    private double[] cameraLocation = { -8.25, -2.875};

    public CameraSubsystem(WebcamName webcamName) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(false)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(getCurrentGameTagLibrary())
                .setLensIntrinsics(510.441, 510.441, 336.87, 250.008)
                .build();

        blueBlobDetector = new ColorBlobDetector(BLUE);
        redBlobDetector = new ColorBlobDetector(RED);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(blueBlobDetector)
                .addProcessor(redBlobDetector)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        visionPortal.setProcessorEnabled(blueBlobDetector, false);
        visionPortal.setProcessorEnabled(redBlobDetector, false);
    }

    public void setColorBlobDetector(ColorBlobDetector.PropColor color) {
        currentColor = color;

        if (color == null) {
            visionPortal.setProcessorEnabled(redBlobDetector, false);
            visionPortal.setProcessorEnabled(blueBlobDetector, false);
            return;
        }

        if (color == BLUE) {
            visionPortal.setProcessorEnabled(redBlobDetector, false);
            visionPortal.setProcessorEnabled(blueBlobDetector, true);
        } else if (color == RED) {
            visionPortal.setProcessorEnabled(redBlobDetector, true);
            visionPortal.setProcessorEnabled(blueBlobDetector, false);
        }
    }

    public void pauseCamera() {
        visionPortal.stopStreaming();
    }

    /**
     * Restarts the camera stream. <b>This method does not return until the camera is active and streaming</b>
     */
    public void restartCamera() {
        visionPortal.resumeStreaming();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public boolean isPortalStreaming() {
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public Pose2d initFindPosition() {
        return getPoseFromAprilTag();
    }

    /**
     * Returns Absolute Location Based on AprilTags
     **/
    @Nullable
    public Pose2d getPoseFromAprilTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

//        if(detections.isEmpty()) return null;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                //Return the first global pose we can find
                Pose2d pose = translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
//                return pose;
                Pose2d aprilPose = PoseSupply.values()[detection.id].globalPose;
                Pose2d cameraGlobal = new Pose2d(aprilPose.getX() + -pose.getX(), aprilPose.getY() + pose.getY(), pose.getRotation());
                Pose2d robotGlobal = translateCamToCenter(cameraGlobal);
                return robotGlobal;
            }
        }

        return null;
    }

    /**
     * Returns a <b>relative</b> pose to the AprilTag
     *
     * @param preferred The preferred AprilTag to use
     * @return The relative pose to the preferred AprilTag.
     * If the preferred AprilTag is not found, the first AprilTag in the secondary list will be used,
     * and it will attempt to translate into a relevant pose to the preferred AprilTag. If none are found,
     * null will be returned.
     */
    @Nullable
    public Pose2d getRelativeAprilTagPose(PoseSupply preferred) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        //First, look for the preferred AprilTag
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (detection.id == preferred.id) {
                    return translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
                }
            }
        }

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (preferred.substitutions.contains(detection.id)) {
                    Pose2d temp = translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
                    return new Pose2d(PoseSupply.values()[detection.id].globalPose.getX() - preferred.globalPose.getX() + temp.getX(), temp.getY(), temp.getRotation());
                }
            }
        }
        return null;
    }

    private Pose2d translateCamToCenter(Pose2d cam) {
        double theta = cam.getHeading();
//        double x = cam.getX() - (cameraLocation[0] * cos(theta)) + (cameraLocation[1] * sin(theta));
//        double y = cam.getY() + (cameraLocation[1] * sin(theta)) + (cameraLocation[0] * cos(theta));
        double d = hypot(cameraLocation[0], cameraLocation[1]);
        double x = cam.getX() + d * cos(theta);
        double y = cam.getY() + d * sin(theta);
        return new Pose2d(x, y, cam.getRotation());
    }

    private Pose2d translateToCam(double x, double y, double z, double yaw, double pitch, double roll) {
        yaw = Math.toRadians(yaw);
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        RealMatrix yawMatrix = new Array2DRowRealMatrix(new double[][]{
                {cos(yaw), -sin(yaw), 0},
                {sin(yaw), cos(yaw), 0},
                {0, 0, 1}
        });
        RealMatrix pitchMatrix = new Array2DRowRealMatrix(new double[][]{
                {cos(pitch), 0, sin(pitch)},
                {0, 1, 0},
                {-sin(pitch), 0, cos(pitch)}
        });
        RealMatrix rollMatrix = new Array2DRowRealMatrix(new double[][]{
                {1, 0, 0},
                {0, cos(roll), -sin(roll)},
                {0, sin(roll), cos(roll)}
        });
        RealMatrix rotationMatrix = yawMatrix.multiply(pitchMatrix).multiply(rollMatrix);

        RealMatrix tAprilToCamMatrix = new Array2DRowRealMatrix(new double[][]{
                {rotationMatrix.getEntry(0, 0), rotationMatrix.getEntry(0, 1), rotationMatrix.getEntry(0, 2), x},
                {rotationMatrix.getEntry(1, 0), rotationMatrix.getEntry(1, 1), rotationMatrix.getEntry(1, 2), y},
                {rotationMatrix.getEntry(2, 0), rotationMatrix.getEntry(2, 1), rotationMatrix.getEntry(2, 2), z},
                {0, 0, 0, 1}
        });

        RealMatrix tCamToAprilMatrix = MatrixUtils.inverse(tAprilToCamMatrix);

        //What threshold to use? Not sure if it matters
        Rotation rotationExtract = new Rotation(tCamToAprilMatrix.getSubMatrix(0, 2, 0, 2).getData(), 0.1);

        double[] angles = rotationExtract.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR);

        return new Pose2d(
                tCamToAprilMatrix.getEntry(1, 3),
                tCamToAprilMatrix.getEntry(0, 3),
                new Rotation2d(angles[0])
        );
    }

    public ColorBlobDetector.PropGuess getPropGuess() {
        switch (currentColor) {
            case RED:
                return redBlobDetector.guess;
            case BLUE:
                return blueBlobDetector.guess;
            default:
                return ColorBlobDetector.PropGuess.UNKNOWN;
        }
    }

}
