package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.pathType.*;


public class pathMan {
    private float currentX, currentY, startX, startY, endX, endY, currentRot;

    public pathMan(pathType PathType, float endXPoint, float endYPoint) {
        this.currentX = xLoc;
        this.currentY = yLoc;
        this.currentRot = rotation;
    }

    public pathMan(pathType PathType, float endXPoint, float endYPoint, float angle) {

    }



}

enum pathType {
    STRAIGHT, STRAIGHT_NO_TURN, STRAIGHT_TURN_TO, ARC_TURN_TO, ARC_NO_TURN
}

