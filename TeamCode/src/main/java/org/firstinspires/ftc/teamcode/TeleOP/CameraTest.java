package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Subsystems.ColorBlobDetector;
import org.firstinspires.ftc.teamcode.Subsystems.PoseSupply;
import org.firstinspires.ftc.teamcode.Subsystems.Robot2023;

@TeleOp(name = "Camera Test", group = "")
public class CameraTest extends OpMode {

    Robot2023 robot = new Robot2023();
    FtcDashboard dash = FtcDashboard.getInstance();
    Pose2d realPose = new Pose2d();
    Pose2d pose2 = new Pose2d();

    @Override
    public void init() {
        robot.init(hardwareMap, true, null);
        //robot.cameraSubsystem.setColorBlobDetector(ColorBlobDetector.PropColor.RED);
    }

    @Override
    public void init_loop() {
//        telemetry.addData("Color Guess", robot.cameraSubsystem.getPropGuess());
        Pose2d pose = robot.cameraSubsystem.getPoseFromAprilTag();
        Pose2d newPose = robot.cameraSubsystem.getRelativeAprilTagPose(PoseSupply.APRILTAG_BLUE_AUDIENCE_BIG);
        if (pose != null) realPose = pose;
        if (newPose != null) pose2 = newPose;
        telemetry.addData("X", realPose.getX()).addData("Y", realPose.getY()).addData("Theta", realPose.getHeading());
        telemetry.addData("X Rel", pose2.getX()).addData("Y", pose2.getY()).addData("Theta", pose2.getHeading());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Color Guess", robot.cameraSubsystem.getPropGuess());
    }
}
