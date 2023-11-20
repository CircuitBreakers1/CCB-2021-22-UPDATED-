package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;
import static java.lang.Math.cos;
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

import java.util.ArrayList;
import java.util.List;

import javax.annotation.Nullable;

public class CameraSubsystem {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ColorBlobDetector colorBlobDetector;

    public CameraSubsystem(WebcamName webcamName, ColorBlobDetector.PropColor color) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(false)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(getCurrentGameTagLibrary())
                .setLensIntrinsics(520.549, 520.549, 313.018, 237.164)
                .build();

        colorBlobDetector = new ColorBlobDetector(color);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorBlobDetector)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }


    public Pose2d initFindPosition() {
        return getPoseFromAprilTag();
    }

    /** NOT ABSOLUTE YET **/ //TODO: FIX
    @Nullable
    public Pose2d getPoseFromAprilTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection: detections) {
            if (detection.metadata != null) {
                //Return the first global pose we can find
                return translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
            }
        }

        return null;
    }

    /**
     * Returns a <b>relative</b> pose to the AprilTag
     * @param preferred The preferred AprilTag to use
     * @param secondary List of acceptable AprilTags to use if the preferred one is not found
     * @return The relative pose to the preferred AprilTag.
     *          If the preferred AprilTag is not found, the first AprilTag in the secondary list will be used,
     *          and it will attempt to translate into a relevant pose to the preferred AprilTag. If none are found,
     *          null will be returned.
     */
    @Nullable
    public Pose2d getRelativeAprilTagPose(PoseSupply preferred) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        //First, look for the preferred AprilTag
        for (AprilTagDetection detection: detections) {
            if (detection.metadata != null) {
                if(detection.id == preferred.id) {
                    return translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
                }
            }
        }

        for(AprilTagDetection detection: detections) {
            if(detection.metadata != null) {
                if(preferred.substitutions.contains(detection.id)) {
                    Pose2d temp = translateToCam(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll);
                    return new Pose2d(temp.minus(preferred.globalPose).getTranslation(), temp.getRotation());
                }
            }
        }
        return null;
    }


    private Pose2d translateToCam(double x, double y, double z, double yaw, double pitch, double roll) {
        yaw = Math.toRadians(yaw);
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        RealMatrix yawMatrix = new Array2DRowRealMatrix(new double[][] {
                {cos(yaw), -sin(yaw), 0},
                {sin(yaw), cos(yaw), 0},
                {0, 0, 1}
        });
        RealMatrix pitchMatrix = new Array2DRowRealMatrix(new double[][] {
                {cos(pitch), 0, sin(pitch)},
                {0, 1, 0},
                {-sin(pitch), 0, cos(pitch)}
        });
        RealMatrix rollMatrix = new Array2DRowRealMatrix(new double[][] {
                {1, 0, 0},
                {0, cos(roll), -sin(roll)},
                {0, sin(roll), cos(roll)}
        });
        RealMatrix rotationMatrix = yawMatrix.multiply(pitchMatrix).multiply(rollMatrix);

        RealMatrix tAprilToCamMatrix = new Array2DRowRealMatrix(new double[][] {
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
}
