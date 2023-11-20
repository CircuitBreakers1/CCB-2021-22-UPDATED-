package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;

public enum PoseSupply {
    ODOMETRY(-1, new Pose2d()),
    //The AprilTags located on the backdrop are not measured exactly but are measured relative to each other. The Pose2d for these is used ONLY for relative between the substitutions
    APRILTAG_BLUE_LEFT(1, new Pose2d(-6,0, new Rotation2d(0)),2, 3), APRILTAG_BLUE_MIDDLE(2, new Pose2d(0,0, new Rotation2d(0)), 1, 3), APRILTAG_BLUE_RIGHT(3, new Pose2d(6,0, new Rotation2d(0)), 1, 2),
    APRILTAG_RED_LEFT(4, new Pose2d(-6,0, new Rotation2d(0)), 5, 6), APRILTAG_RED_MIDDLE(5, new Pose2d(0,0, new Rotation2d(0)), 4, 6), APRILTAG_RED_RIGHT(6, new Pose2d(6,0, new Rotation2d(0)), 4, 6);

    int id;
    ArrayList<Integer> substitutions;
    Pose2d globalPose;

    PoseSupply(int id, Pose2d globalPose, int... subs) {
        this.id = id;
        this.globalPose = globalPose;
        for (int i : subs) {
            this.substitutions.add(i);
        };
    }
}
