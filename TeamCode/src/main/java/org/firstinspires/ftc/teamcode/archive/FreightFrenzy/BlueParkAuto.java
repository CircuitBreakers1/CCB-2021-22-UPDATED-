/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.archive.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="BlueParkAuto", group ="Blue")
@Disabled

public class BlueParkAuto extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    HardwareInit robot = new HardwareInit();

    private static final String VUFORIA_KEY =
            "AWC3x6z/////AAABmVlXzJgJHEkClTfzpPhSQSAOSo2ALGWXmreVgLVShBXUJg8BGyNP06zZuMyV0UZUcxC2xqq5jFsSEg1V0yYBBfvKinPneqTDkbkGA1vDE18L884DGyo3awssbrJEnYxMlTYnqT6HAsQO1SQ+DiTDRJOkI2Bo8rmK2mXLXaZPApKXptVgvEFUds0cNi1DZX3d8BzNxmQuIgT9jY+4L5B0sUnEJyZEyiwKqUhpGDmWNQd3yzQcdI9vFyyX6/4FrK6GaT65uV5xW1v4dwvyZite2Fkd0/6J403Wyy3hXBBvsvUZLJvEMWa42Q31/RUDXbaJyric+SOOU1QGFOTEmN4yt7o3hgO4R/SoyWtadjNI0qx6";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private final float Xline = 165;
    private int labelCount;

    private final float posX = 0;
    private final float posY = 0;
    private final float posAngle = 0;

    private VuforiaLocalizer vuforia    = null;
    private VuforiaLocalizer.Parameters parameters = null;
    private VuforiaTrackables targets   = null;
    private WebcamName webcamName       = null;
    private List<VuforiaTrackable> allTrackables = null;
    private static final String TFOD_MODEL_ASSET = "CcbBee.tflite";
    private static final String[] LABELS = {"Bee"};
    private TFObjectDetector tfod;


    private int scanCount = 0; //Used to remove unneeded checks when play is pressed
    private int targetLevel;
    private final float[] beeLeft = new float[2];
    private final float[] beeTop = new float[2];
    private int recCount = 0;
    private int screenHeight;
    private final boolean isBee = false;
    private boolean actualBee = false;

    @Override public void runOpMode() {
        robot.init(hardwareMap);
        initImu();
        initVuforia();
        initTfod();

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        while (!isStarted()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                recCount = 0;
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                labelCount = updatedRecognitions.size();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    //if(recognition.getBottom() < 480) {
                    screenHeight = recognition.getImageHeight();
                    beeLeft[recCount] = recognition.getLeft();
                    beeTop[recCount] = recognition.getTop();
                    recCount++;
                    //isBee = true;

                    //}

                    i++;
                }

            }
            scanCount++;

            if(!(labelCount == 0)) {
                int j;
                for(j = 0; j<recCount; j++) {
                    if(!(beeTop[j] < (0.25 * screenHeight))) {
                        actualBee = true;
                        if(beeLeft[j] > Xline) {
                            telemetry.addData("Target Guess:", "Right");
                            telemetry.addData("Target Level:", "Top");
                            targetLevel = 3;
                        } else {
                            telemetry.addData("Target Guess:", "Middle");
                            telemetry.addData("Target Level:", "Middle");
                            targetLevel = 2;
                        }
                    }

                }
                if(!actualBee) {
                    telemetry.addData("Target Guess:", "Left");
                    telemetry.addData("Target Level:", "Bottom");
                    targetLevel = 1;

                }
            } else {
                telemetry.addData("Target Guess:", "Left");
                telemetry.addData("Target Level:", "Bottom");
                targetLevel = 1;
            }


            telemetry.update();
        }

        waitForStart();

        if(scanCount < 5) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                recCount = 0;
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                labelCount = updatedRecognitions.size();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    screenHeight = recognition.getImageHeight();
                    beeLeft[recCount] = recognition.getLeft();
                    beeTop[recCount] = recognition.getTop();
                    recCount++;

                    i++;
                }
            }

            if(!(labelCount == 0)) {
                int j;
                for(j = 0; j<recCount; j++) {
                    if(!(beeTop[j] < (0.25 * screenHeight))) {
                        actualBee = true;
                        if(beeLeft[j] > Xline) {
                            telemetry.addData("Target Guess:", "Right");
                            telemetry.addData("Target Level:", "Top");
                            targetLevel = 3;
                        } else {
                            telemetry.addData("Target Guess:", "Middle");
                            telemetry.addData("Target Level:", "Middle");
                            targetLevel = 2;
                        }
                    }

                }
                if(!actualBee) {
                    telemetry.addData("Target Guess:", "Left");
                    telemetry.addData("Target Level:", "Bottom");
                    targetLevel = 1;

                }
            } else {
                telemetry.addData("Target Guess:", "Left");
                telemetry.addData("Target Level:", "Bottom");
                targetLevel = 1;
            }


            telemetry.update();
        }


        telemetry.addData("Status", "Moving off wall...");
        telemetry.update();
        moveIN(6,0.5);
        gyroTurn(20,0.5);

        if(targetLevel == 1) {
            robot.rightArm.setTargetPosition(51);
            robot.leftArm.setTargetPosition(51);
        } else if(targetLevel == 2) {
            robot.rightArm.setTargetPosition(90);
            robot.leftArm.setTargetPosition(90);
        } else {
            robot.rightArm.setTargetPosition(140);
            robot.leftArm.setTargetPosition(140);
        }

        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightArm.setPower(1);
        robot.leftArm.setPower(1);

        moveIN(15, 0.5);

        sleep(1000);

        robot.intake.setPower(0.5);
        sleep(2000);
        robot.intake.setPower(0);

        moveIN(-3, 0.25);

        robot.rightArm.setPower(0);
        robot.leftArm.setPower(0);

        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyroTurn(70, 0.5);
        moveIN(-56, 1);
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void initImu() {
        BNO055IMU.Parameters Parameters = new BNO055IMU.Parameters();
        Parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Parameters.loggingEnabled      = true;
        Parameters.loggingTag          = "IMU";
        Parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Turn a specified number of degrees using the IMU
     * @param degrees Positive degrees moves in a clockwise direction
     * @param speed Speed to run the motors at
     */
    public void gyroTurn(double degrees, double speed) {
        if(!opModeIsActive()) {
            return;
        }

        double startDegrees = angles.firstAngle;
        double targetDegrees = startDegrees - degrees;
        if(degrees < 0) {
            robot.rightBack.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(-speed);
            robot.leftFront.setPower(-speed);
        } else {
            robot.rightBack.setPower(-speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(speed);
            robot.leftFront.setPower(speed);
        }
        while(!(angles.firstAngle + 3 > targetDegrees && angles.firstAngle - 3 < targetDegrees)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            sleep(10);
        }

        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
    }

    /**
     * Moves a distance using encoders
     * @param inches
     * @param speed
     */
    public void moveIN(double inches, double speed) {
        if(!opModeIsActive()) {
            return;
        }

        double wheelCircumference = 4 * 3.14;
        double ticksPerRot = 537;
        double rotations = inches / wheelCircumference;
        double ticks = ticksPerRot * rotations;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftFront.setTargetPosition((int) ticks);
        robot.rightFront.setTargetPosition((int) ticks);
        robot.leftBack.setTargetPosition((int) ticks);
        robot.rightBack.setTargetPosition((int) ticks);

        robot.rightBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.leftFront.setPower(speed);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(((robot.leftBack.isBusy() && robot.leftFront.isBusy()) || (robot.rightBack.isBusy() && robot.rightFront.isBusy())) && opModeIsActive()) {
            /*
             * Robot gets some free time.
             * What does it do during it's free time?
             * No one knows.
             * Maybe it's one of those chess bots
             * or perhaps it's training a neural network
             * through a harsh yet effective process of generational evolution.
             * It probably just naps. It works pretty hard.
             * Even though we might not know what it does, all that matters to us is that it comes
             * back when the motors are done.
             */
        }

        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveIN(double inches, double speed, float timeoutMillis) {
        if(!opModeIsActive()) {
            return;
        }

        double wheelCircumference = 4 * 3.14;
        double ticksPerRot = 537;
        double rotations = inches / wheelCircumference;
        double ticks = ticksPerRot * rotations;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftFront.setTargetPosition((int) ticks);
        robot.rightFront.setTargetPosition((int) ticks);
        robot.leftBack.setTargetPosition((int) ticks);
        robot.rightBack.setTargetPosition((int) ticks);

        robot.rightBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.leftFront.setPower(speed);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        float startMillis = System.currentTimeMillis();
        boolean haveTime = true;

        while
        (((robot.leftBack.isBusy() && robot.leftFront.isBusy()) || (robot.rightBack.isBusy() && robot.rightFront.isBusy())) && opModeIsActive() && haveTime) {
            sleep(10);
            if(System.currentTimeMillis() >   (startMillis + timeoutMillis)) {
                haveTime = false;
            }
        }

        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initVuforia() {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);


        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.375f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 10.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -8.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 0, 90));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
    }
}
