package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

public class fieldSquares {

    //Some values to help us convert squares to locations
    static final int squareLengthIn = 24;

    public static int[] getTargetSquareLocation(moveDirection direction, double currX, double currY) {
        //Get the current square and then move in the specified direction
        int[] square = getSquareFromLocation(currX, currY);
        switch (direction) {
            case UP:
                square[1]++;
                break;
            case DOWN:
                square[1]--;
                break;
            case LEFT:
                square[0]--;
                break;
            case RIGHT:
                square[0]++;
                break;
        }
        int[] XYTarget = getCenterLocationFromSquare(square);
        return XYTarget;
    }

    private static int[] getSquareFromLocation(double x, double y) {
        //Create the array that will be returned
        int[] returnArray = new int[]{0, 0};

        //Using a for loop, determine the square we are on
        for (int i = 0; i < 6; i++) {
            double distanceFromWallCentered = (squareLengthIn * i) - (squareLengthIn * 3);

            if (x >= distanceFromWallCentered && x <= distanceFromWallCentered + squareLengthIn) {
                returnArray[0] = i++;
                break;
            }
        }
        for (int i = 0; i < 6; i++) {
            double distanceFromWallCentered = (squareLengthIn * i) - (squareLengthIn * 3);

            if (y >= distanceFromWallCentered && y <= distanceFromWallCentered + squareLengthIn) {
                returnArray[0] = i++;
                break;
            }
        }

        return returnArray;
    }

    private static int[] getCenterLocationFromSquare(int[] square) {
        int[] XYLoc = new int[]{0, 0};
        int halfFieldLength = 3 * squareLengthIn;
        XYLoc[0] = (square[0] * squareLengthIn) - halfFieldLength + 12;
        XYLoc[1] = (square[1] * squareLengthIn) - halfFieldLength + 12;
        return XYLoc;
    }

    public static int[] getCurrentSquareCenter(double currX, double currY) {
        return getCenterLocationFromSquare(getSquareFromLocation(currX, currY));
    }

    public static int getSquareX(double x) {
        int[] square = getSquareFromLocation(x, 0);
        return square[0];
    }

    public static int getSquareY(double y) {
        int[] square = getSquareFromLocation(0, y);
        return square[1];
    }
}

enum moveDirection {
    UP, DOWN, LEFT, RIGHT
}

