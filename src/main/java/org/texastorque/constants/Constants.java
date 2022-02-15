package org.texastorque.constants;

public class Constants {
    // Conversions
    public static final double FOOT_TO_METER = 0.3048;
    public static final double INCH_TO_FOOT = 1. / 12.;

    // Drivebase
    public static final double DRIVE_WHEEL_RADIUS_METERS = 1.788 * INCH_TO_FOOT * FOOT_TO_METER; // 1.788 is width with
                                                                                                 // wear
    public static final double DRIVE_MAX_SPEED_METERS = 5;
    public static final double DRIVE_MAX_ANGUAR_SPEED_RADIANS = 4 * Math.PI;

    public static final double DISTANCE_TO_CENTER_X = 10.875 * INCH_TO_FOOT * FOOT_TO_METER;
    public static final double DISTANCE_TO_CENTER_Y = 10.875 * INCH_TO_FOOT * FOOT_TO_METER;

    public static final double ROTATE_MANAGER_PID_P = .6;
    public static final double ROTATE_MANAGER_PID_I = 0;
    public static final double ROTATE_MANAGER_PID_D = 0;

    public static final double DRIVE_Ks = 0.37843;
    public static final double DRIVE_Kv = 1.5423;
    public static final double DRIVE_Ka = 1.5065;
    public static final double DRIVE_Kp = 0;

    // Magazine
    public static final double MAGAZINE_BELT_SPEED = 1.;

    // Intake
    public static final double INTAKE_ROTARY_SPEED = 1.;

    // Shooter
    public static final double FLYWHEEL_Kv = 0.14068; // Values are for rotations/sec
    public static final double FLYWHEEL_Ka = 0.026483;
    public static final double FLYWHEEL_Ks = 0.30442;
    public static final double FLYWHEEL_Kp = 0.076869;
    public static final double FLYWHEEL_Ki = 0;
    public static final double FLYWHEEL_Kd = 0;
    public static final double HOOD_MIN = 0;
    public static final double HOOD_MAX = 50;

    // Climber
    public static final double CLIMBER_SPEED = .5;
    public static final double CLIMBER_LEFT_LIMIT_HIGH = 102;
    public static final double CLIMBER_RIGHT_LIMIT_HIGH = -102; // Will change again
    public static final double CLIMBER_LEFT_LIMIT_LOW = 0;
    public static final double CLIMBER_RIGHT_LIMIT_LOW = 0;

    // Turret
    public static final double TURRET_Ks = -1;
    public static final double TURRET_Kv = -1;
    public static final double TURRET_Ka = -1;
    public static final double TURRET_Kp = -1;
    public static final double TURRET_Ki = -1;
    public static final double TURRET_Kd = -1;
    public static final double HEIGHT_TO_LIMELIGHT_METERS = 31.9694 * INCH_TO_FOOT * FOOT_TO_METER;
    public static final double TURRET_RATIO = 192.708; // to 1

    // Information
    public static final double TOP_SPEED_FEET = 16.52;
    public static final double TOP_SPEED_METERS = TOP_SPEED_FEET * FOOT_TO_METER;
    public static final double TOP_ACCELERATION_METERS = 1;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI;

    // Path Planner
    public static final double PATH_PLANNER_X_P = 0;
    public static final double PATH_PLANNER_X_I = 0;
    public static final double PATH_PLANNER_X_D = 0;

    public static final double PATH_PLANNER_Y_I = 0;
    public static final double PATH_PLANNER_Y_P = 0;
    public static final double PATH_PLANNER_Y_D = 0;

    public static final double PATH_PLANNER_R_P = 0;
    public static final double PATH_PANNER_R_I = 0;
    public static final double PATH_PLANNER_R_D = 0;

}
