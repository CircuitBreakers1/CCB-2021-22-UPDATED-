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

package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import org.firstinspires.ftc.teamcode.archive.FreightFrenzy.HardwareInit;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name="ttttttrt ", group ="Red")
public class vuCodeLib extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    HardwareInit robot = new HardwareInit();

    private static final String VUFORIA_KEY =
            "AWC3x6z/////AAABmVlXzJgJHEkClTfzpPhSQSAOSo2ALGWXmreVgLVShBXUJg8BGyNP06zZuMyV0UZUcxC2xqq5jFsSEg1V0yYBBfvKinPneqTDkbkGA1vDE18L884DGyo3awssbrJEnYxMlTYnqT6HAsQO1SQ+DiTDRJOkI2Bo8rmK2mXLXaZPApKXptVgvEFUds0cNi1DZX3d8BzNxmQuIgT9jY+4L5B0sUnEJyZEyiwKqUhpGDmWNQd3yzQcdI9vFyyX6/4FrK6GaT65uV5xW1v4dwvyZite2Fkd0/6J403Wyy3hXBBvsvUZLJvEMWa42Q31/RUDXbaJyric+SOOU1QGFOTEmN4yt7o3hgO4R/SoyWtadjNI0qx6";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private final float Xline = 165;
    private int labelCount;

    private float posX = 0;
    private float posY = 0;
    private float posAngle = 0;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaLocalizer.Parameters parameters = null;
    private VuforiaTrackables targets   = null;
    private WebcamName webcamName       = null;
    private List<VuforiaTrackable> allTrackables = null;
    private static final String TFOD_MODEL_ASSET = "Bee 2.0.tflite";
    private static final String[] LABELS = {"Bee 2.0"};
    private TFObjectDetector tfod;


    private boolean targetVisible       = false;
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
                    //beeLeft = recognition.getLeft();

                    i++;
                }

            }
            scanCount++;
            /*
            if(labelCount == 0) {
                telemetry.addData("Target Guess:", "Left");
                telemetry.addData("Target Level:", "Bottom");
                targetLevel = 1;
            } else if(beeLeft > Xline) {
                telemetry.addData("Target Guess:", "Right");
                telemetry.addData("Target Level:", "Top");
                targetLevel = 3;
            } else {
                telemetry.addData("Target Guess:", "Middle");
                telemetry.addData("Target Level:", "Bottom");
                targetLevel = 2;
            }
            telemetry.update(); */
        }


        telemetry.addData("Status", "Moving off wall...");
        telemetry.update();
        moveIN(6,0.5);
        telemetry.addData("Status:", "Turning");
        telemetry.update();
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

        moveIN(-2, 0.25);

        robot.rightArm.setPower(0);
        robot.leftArm.setPower(0);

        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveIN(-7, 0.25);
        gyroTurn(50, 0.5);
        robot.backSpinner.setPower(-0.4);
        moveIN(-27.5, 0.25, 6000);
        telemetry.addData("Status", "duck");
        telemetry.update();
        sleep(4000);
        robot.backSpinner.setPower(0);

        moveIN(105, 1);



        /*
        while (!isStopRequested()) {
            //updateLocation();
            idle();
        }
        */
        /*
        while (!isStopRequested()) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                posX = translation.get(0) / mmPerInch;
                posY = translation.get(1) / mmPerInch;
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                posAngle = rotation.thirdAngle;
                telemetry.addData("Pos X", posX);
                telemetry.addData("Pos Y", posY);
                telemetry.addData("Angle", posAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

            if(!lol) {
                lol=true;
                goToPosition(0,0,0.5);
            }


            //float[] coords = lastLocation.getTranslation().getData();
            //posX = coords[0];
            //posY = coords[1];
            //posAngle = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
        }


        // Disable Tracking when we are done;
        targets.deactivate(); */
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
     * Turns to a direction based on the field itself
     * @param degrees Absolute direction
     * @param speed
     * @param absolute Passing false runs relative gyro turn and turns a number of degrees
     */
    public void gyroTurn(double degrees, double speed, boolean absolute) {
        if(!absolute) {
            gyroTurn(degrees, speed);
            return;
        }
        double turnAmount = posAngle - degrees;
        gyroTurn(turnAmount, speed);
    }

    public void turnToAngle(double targetAngle, double speed) {
        double startDegrees = angles.firstAngle;
        /*
        double zeroedTarget = targetAngle - startDegrees;
        boolean turnRight;
        if(zeroedTarget > 0) {
            turnRight = false;
        } else {
            turnRight = true;
        }
        if(Math.abs(zeroedTarget) > 180) {
            turnRight = !turnRight;
        } */

        double angularMovement = targetAngle + startDegrees;
        if(Math.abs(angularMovement) > 180) {

        }
        gyroTurn(angularMovement, speed);
    }

    /**
     * Moves a distance using encoders
     * @param inches
     * @param speed
     */
    public void moveIN(double inches, double speed) {
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

    /**
     * Goes to a position on the field
     * @param targetX X coord to go to
     * @param targetY Y coord to go to
     * @param speed
     */
    public void goToPosition(float targetX, float targetY, double speed) {
        updateLocation();

        float currentX = posX;
        float currentY = posY;

        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;

        double targetRad = Math.atan(deltaX/deltaY);
        double targetDegrees = Math.toDegrees(targetRad);

        gyroTurn(targetDegrees, speed, true);

        double z = Math.sqrt(((deltaX * deltaX) + (deltaY * deltaY)));
        moveIN(z,speed);

        updateLocation(targetX, targetY, (float)targetDegrees);
    }

    /**
     * Updates location
     */
    private void updateLocation() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            posX = translation.get(0) / mmPerInch;
            posY = translation.get(1) / mmPerInch;
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            posAngle = rotation.thirdAngle;
            telemetry.addData("Pos X", posX);
            telemetry.addData("Pos Y", posY);
            telemetry.addData("Angle", posAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
        /*
        float[] coords = lastLocation.getTranslation().getData();
        posX = coords[0];
        posY = coords[1];
        posAngle = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
        */
    }


    //TODO: Automatic angle estimation?
    /**
     * Updates location. If you know some values you can set them, however they will be overridden
     * if a target is visible. If no target is visible and a zero is passed the value will not be
     * updated.
     * @param estX Input a exact zero if no value is known, regularly generated numbers shouldn't
     *             be affected due to the decimal precision making it nearly impossible to be zero
     * @param estY Same as estX
     * @param estAngle Same as estX
     */
    public void updateLocation(float estX, float estY, float estAngle) {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
                break;
            }
        }
        float[] coords = lastLocation.getTranslation().getData();
        if (targetVisible) {
            posX = coords[0];
            posY = coords[1];
            posAngle = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
            return;
        }
        if(estX != 0) {
            posX = estX;
        }
        if(estY != 0) {
            posY = estY;
        }
        if(estAngle != 0) {
            posAngle = estAngle;
        }
    }

    public void initVuforia() {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
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

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

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
