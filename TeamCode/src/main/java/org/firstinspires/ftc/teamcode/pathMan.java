package org.firstinspires.ftc.teamcode;

public class pathMan {
    private float y, x;

    public pathMan(pathType PathType, float endXPoint, float endYPoint) {

    }

    public pathMan(pathType PathType, float endXPoint, float endYPoint, float angle) {

    }



}

enum pathType {
    STRAIGHT, STRAIGHT_NO_TURN, STRAIGHT_TURN_TO, ARC_TURN_TO, ARC_NO_TURN
}

