package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.NewRobot2023.holOdom;
import static java.lang.Math.PI;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class NewAutoSwitcher {
    public enum ConfigSetting {
        START_LOCATION,
        PARK_LOCATION,
        MOVEMENT_PATH,
        PLACE_LOCATION,
        DELAY_SECONDS;

        public static ConfigSetting cycleUp(ConfigSetting setting) {
            switch (setting) {
                case START_LOCATION:
                    return PARK_LOCATION;
                case PARK_LOCATION:
                    return MOVEMENT_PATH;
                case MOVEMENT_PATH:
                    return PLACE_LOCATION;
                case PLACE_LOCATION:
                    return DELAY_SECONDS;
                case DELAY_SECONDS:
                    return START_LOCATION;
            }
            return START_LOCATION;
        }

        public static ConfigSetting cycleDown(ConfigSetting setting) {
            switch (setting) {
                case START_LOCATION:
                    return DELAY_SECONDS;
                case PLACE_LOCATION:
                    return MOVEMENT_PATH;
                case PARK_LOCATION:
                    return START_LOCATION;
                case MOVEMENT_PATH:
                    return PARK_LOCATION;
                case DELAY_SECONDS:
                    return PLACE_LOCATION;

            }
            return START_LOCATION;
        }
    }

    public enum StartLocation {
        //Assume robot is 17" long and 17" wide. Start centered on tile, touching wall
        RED_BACKDROP(12, -63.5, PI / 2),
        RED_AUDIENCE(-36, -63.5, PI / 2),
        BLUE_BACKDROP(12, 63.5, -PI / 2),
        BLUE_AUDIENCE(-36, 63.5, -PI / 2);

        public final Pose2d startPose;

        StartLocation(double x, double y, double theta) {
            startPose = new Pose2d(x, y, new Rotation2d(theta));
        }
    }

    public enum ParkLocation {
        INSIDE, OUTSIDE
    }

    public enum Alliance {
        RED, BLUE
    }

    public enum MovementPath {
        INSIDE_TRUSS,
        OUTSIDE_TRUSS,
        STAGE_DOOR
    }

    public enum PlaceLocation {
        LEFT, RIGHT
    }

    public enum SwitchType {
        MIRROR_X_AXIS, MIRROR_X_AXIS_AND_START_Y
    }

    private StartLocation startLocation;
    private ParkLocation parkLocation;
    private MovementPath movementPath;
    private Alliance alliance;
    private PlaceLocation placeLocation;
    private int delaySec;
    private int delayMin = 0, delayMax = 10;
    private final NewMovementSubsystem movementSubsystem;

    public NewAutoSwitcher(NewMovementSubsystem movementSubsystem) {
        startLocation = StartLocation.BLUE_BACKDROP;
        alliance = Alliance.BLUE;
        placeLocation = PlaceLocation.LEFT;
        parkLocation = ParkLocation.OUTSIDE;
        movementPath = MovementPath.STAGE_DOOR;
        delaySec = 0;
        this.movementSubsystem = movementSubsystem;
    }

    public String get(ConfigSetting setting) {
        switch (setting) {
            case START_LOCATION:
                return startLocation.toString();
            case PARK_LOCATION:
                return parkLocation.toString();
            case MOVEMENT_PATH:
                return movementPath.toString();
            case PLACE_LOCATION:
                return placeLocation.toString();
            case DELAY_SECONDS:
                return String.valueOf(delaySec);
        }
        return "";
    }

    public StartLocation getStartLocation() {
        return startLocation;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public ParkLocation getParkLocation() {
        return parkLocation;
    }

    public MovementPath getMovementPath() {
        return movementPath;
    }

    public PlaceLocation getPlaceLocation() {
        return placeLocation;
    }
    public int getDelayMS() {
        return delaySec * 1000;
    }
    public int getDelaySec() {
        return delaySec;
    }

    public void toggleUp(ConfigSetting setting) {
        switch (setting) {
            case START_LOCATION:
                switch (startLocation) {
                    case RED_BACKDROP:
                        startLocation = StartLocation.RED_AUDIENCE;
                        alliance = Alliance.RED;
                        break;
                    case RED_AUDIENCE:
                        startLocation = StartLocation.BLUE_BACKDROP;
                        alliance = Alliance.BLUE;
                        break;
                    case BLUE_BACKDROP:
                        startLocation = StartLocation.BLUE_AUDIENCE;
                        alliance = Alliance.BLUE;
                        break;
                    case BLUE_AUDIENCE:
                        startLocation = StartLocation.RED_BACKDROP;
                        alliance = Alliance.RED;
                        break;
                }
                holOdom.updatePose(startLocation.startPose);
                break;
            case PARK_LOCATION:
                switch (parkLocation) {
                    case INSIDE:
                        parkLocation = ParkLocation.OUTSIDE;
                        break;
                    case OUTSIDE:
                        parkLocation = ParkLocation.INSIDE;
                        break;
                }
                break;
            case MOVEMENT_PATH:
                switch (movementPath) {
                    case INSIDE_TRUSS:
                        movementPath = MovementPath.OUTSIDE_TRUSS;
                        break;
                    case OUTSIDE_TRUSS:
                        movementPath = MovementPath.STAGE_DOOR;
                        break;
                    case STAGE_DOOR:
                        movementPath = MovementPath.INSIDE_TRUSS;
                        break;
                }
                break;
            case PLACE_LOCATION:
                switch (placeLocation) {
                    case LEFT:
                        placeLocation = PlaceLocation.RIGHT;
                        break;
                    case RIGHT:
                        placeLocation = PlaceLocation.LEFT;
                        break;
                }
                break;
            case DELAY_SECONDS:
                if(delaySec < delayMax) delaySec++;
                break;
        }
    }

    public void toggleDown(ConfigSetting setting) {
        switch (setting) {
            case START_LOCATION:
                switch (startLocation) {
                    case RED_BACKDROP:
                        startLocation = StartLocation.BLUE_AUDIENCE;
                        alliance = Alliance.BLUE;
                        break;
                    case RED_AUDIENCE:
                        startLocation = StartLocation.RED_BACKDROP;
                        alliance = Alliance.RED;
                        break;
                    case BLUE_BACKDROP:
                        startLocation = StartLocation.RED_AUDIENCE;
                        alliance = Alliance.RED;
                        break;
                    case BLUE_AUDIENCE:
                        startLocation = StartLocation.BLUE_BACKDROP;
                        alliance = Alliance.BLUE;
                        break;
                }
                holOdom.updatePose(startLocation.startPose);
                break;
            case PARK_LOCATION:
                switch (parkLocation) {
                    case INSIDE:
                        parkLocation = ParkLocation.OUTSIDE;
                        break;
                    case OUTSIDE:
                        parkLocation = ParkLocation.INSIDE;
                        break;
                }
                break;
            case MOVEMENT_PATH:
                switch (movementPath) {
                    case INSIDE_TRUSS:
                        movementPath = MovementPath.STAGE_DOOR;
                        break;
                    case OUTSIDE_TRUSS:
                        movementPath = MovementPath.INSIDE_TRUSS;
                        break;
                    case STAGE_DOOR:
                        movementPath = MovementPath.OUTSIDE_TRUSS;
                        break;
                }
                break;
            case PLACE_LOCATION:
                switch (placeLocation) {
                    case LEFT:
                        placeLocation = PlaceLocation.RIGHT;
                        break;
                    case RIGHT:
                        placeLocation = PlaceLocation.LEFT;
                        break;
                }
                break;
            case DELAY_SECONDS:
                if(delaySec > delayMin) delaySec--;
                break;
        }
    }

    public void moveSwitch(double x, double y, double theta, double maxSpeed, boolean precise, Runnable loop, SwitchType switchType) {
        //Blue side is default, so no need to change anything for it.
        if (alliance == Alliance.RED) {
            switch (switchType) {
                case MIRROR_X_AXIS:
                    y = -y;
                    theta = -theta;
                    break;
                case MIRROR_X_AXIS_AND_START_Y:
                    x = -x;
                    theta = -theta;
                    double yDif = y - startLocation.startPose.getY();
                    double thetaDif = theta - startLocation.startPose.getHeading();
                    y = y - (2 * yDif);
                    theta = theta - (2 * thetaDif);
                    break;
            }
        }
        movementSubsystem.moveTo(x, y, theta, maxSpeed, precise, loop);
    }

    public void moveSwitch(double x, double y, double theta, double maxSpeed, Runnable loop) {
        moveSwitch(x, y, theta, maxSpeed, true, loop, SwitchType.MIRROR_X_AXIS);
    }

    public void moveSwitch(double x, double y, double theta, double maxSpeed, boolean precise, Runnable loop) {
        moveSwitch(x, y, theta, maxSpeed, precise, loop, SwitchType.MIRROR_X_AXIS);
    }

    public void moveSwitch(double x, double y, double theta, double maxSpeed, boolean precise) {
        moveSwitch(x, y, theta, maxSpeed, precise, null, SwitchType.MIRROR_X_AXIS);
    }

    public void moveSwitch(double x, double y, double theta, double maxSpeed) {
        moveSwitch(x, y, theta, maxSpeed, true, null, SwitchType.MIRROR_X_AXIS);
    }
}
