/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.base.TorqueSubsystemState;
import org.texastorque.torquelib.control.TorqueLookUpTable;
import org.texastorque.torquelib.control.TorqueLookUpTable.TorqueDisjointData;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueRollingMedian;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.sensors.TorqueLight;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Shooter extends TorqueSubsystem implements Subsystems {
    private static volatile Shooter instance;

    public static final double HOOD_MIN = 0, HOOD_MAX = 35, ERROR = 50, FLYWHEEEL_MAX = 3000,
            FLYWHEEEL_REDUCTION = 5 / 3., CAMERA_HEIGHT = Units.inchesToMeters(41), HUB_RADIUS = .6778625,
            TURRET_RADIUS = Units.inchesToMeters(7.2),
            TARGET_HEIGHT = 2.6416;

    public static final Rotation2d CAMERA_ANGLE = Rotation2d.fromDegrees(30);

    public static final Translation2d HUB_CENTER_POSITION = new Translation2d(8.2, 4.1),
            CAMERA_TO_ROBOT = new Translation2d(Units.inchesToMeters(2), CAMERA_HEIGHT);

    public static final Pose2d HUB_ORIGIN = new Pose2d(HUB_CENTER_POSITION.getX(),
            HUB_CENTER_POSITION.getY(), new Rotation2d());

    public static TorqueLookUpTable disjointData;
    public static TorqueDisjointData data;
    public static TorqueRollingMedian rollingMedian = new TorqueRollingMedian(10);

    public enum ShooterState implements TorqueSubsystemState {
        OFF,
        IDLE;

        public final boolean isShooting() {
            return this != OFF && this != IDLE;
        }
    }

    private final TorqueLight camera;

    private final TorqueSparkMax hood, flywheelLeft, flywheelRight;

    private double hoodSetpoint, flywheelSpeed, distance;
    private ShooterState state = ShooterState.OFF;

    private Shooter() {
        camera = new TorqueLight("gloworm");

        flywheelLeft = new TorqueSparkMax(Ports.SHOOTER.FLYWHEEL.LEFT);
        flywheelRight = new TorqueSparkMax(Ports.SHOOTER.FLYWHEEL.RIGHT);
        flywheelLeft.configurePID(TorquePID.create(.000005).addFeedForward(.00034).setTolerance(20).build());

        hood = new TorqueSparkMax(Ports.SHOOTER.HOOD);
        hood.configurePID(TorquePID.create(.1)
                .addIntegral(.001)
                .addOutputRange(-.7, .7)
                .addIntegralZone(.3)
                .build());

        hood.configurePositionalCANFrame();
        hood.burnFlash();
    }

    public final void setState(final ShooterState state) {
        this.state = state;
    }

    public final void setHoodPosition(final double hoodSetpoint) {
        this.hoodSetpoint = hoodSetpoint;
    }

    public final void setFlywheelSpeed(final double flywheelSpeed) {
        this.flywheelSpeed = flywheelSpeed;
    }

    public final void setDistance(final double distance) {
        this.distance = distance;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        state = ShooterState.OFF;
    }

    @Override
    public final void update(final TorqueMode mode) {
        distance = TorqueMath.round(getDistance(), 1);
        data = disjointData.calculate(distance);

        camera.update();

        if (state != ShooterState.OFF) {
            flywheelLeft.setVelocityRPM(clampRPM(200));
            flywheelRight.setVoltage(-flywheelLeft.getVoltage());
        } else {
            flywheelLeft.setVoltage(0);
            flywheelRight.setVoltage(0);
        }

        hood.setPosition(clampHood(hoodSetpoint));

        TorqueSubsystemState.logState(state);

        SmartDashboard.putNumber("Distance", distance);

    }

    public final boolean isShooting() {
        return state.isShooting();
    }

    public final boolean isReady() {
        return isShooting() && Math.abs(flywheelSpeed - flywheelLeft.getVelocityRPM() / FLYWHEEEL_REDUCTION) < ERROR;
    }

    private final double clampRPM(final double rpm) {
        return TorqueMath.constrain(rpm, 0, FLYWHEEEL_MAX);
    }

    private final double clampHood(final double hood) {
        return TorqueMath.constrain(hood, HOOD_MIN, HOOD_MAX);
    }

    public final TorqueLight getCamera() {
        return camera;
    }

    public final ShooterState getState() {
        return state;
    }

    public final double getDistance() {
        return rollingMedian
                .calculate(TorqueLight.getDistanceToElevatedTarget(camera, CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_ANGLE));
    }

    public final Pose2d getVisionPositionEstimate() {
        TorqueUtil.notImplemented();

        return new Pose2d();
    }

    public final double calculateDistance() {
        return TorqueLight.getDistanceToElevatedTarget(camera, CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_ANGLE);
    }

    public final double getDistanceVelocity(double distance) {
        double gravity = -9.8;
        double FINAL_ANGLE = 0;
        double velocity = Math.sqrt(Math.sqrt((gravity * distance) / Math.sin(2 * Math.toRadians(FINAL_ANGLE))));
        return 0;
    }

    public final double getTargetOffset() {
        return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("gloworm").getEntry("targetYaw")
                .getNumber(0).doubleValue();
    }
    
    public static final synchronized Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }
}
