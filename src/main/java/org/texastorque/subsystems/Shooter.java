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
import org.texastorque.torquelib.control.TorqueDisjointDataRegression;
import org.texastorque.torquelib.control.TorqueDisjointDataRegression.DisjointData;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueRollingMedian;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.sensors.TorqueLight;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Shooter extends TorqueSubsystem implements Subsystems {
    private static volatile Shooter instance;

    public static final double HOOD_MIN = 0, HOOD_MAX = 40, ERROR = 50, FLYWHEEEL_MAX = 3000, FLYWHEEEL_IDLE = .1,
            FLYWHEEEL_REDUCTION = 5 / 3., CAMERA_HEIGHT = Units.inchesToMeters(33),
            TARGET_HEIGHT = 2.6416;

    public static final double HUB_RADIUS = .6778625;

    public static final double TURRET_RADIUS = Units.inchesToMeters(7.2);

    public static final Rotation2d CAMERA_ANGLE = Rotation2d.fromDegrees(30);

    public static final Translation2d HUB_CENTER_POSITION = new Translation2d(8.2, 4.1);

    public static final Pose2d HUB_ORIGIN = new Pose2d(HUB_CENTER_POSITION.getX(),
            HUB_CENTER_POSITION.getY(), new Rotation2d());

    public static final Translation2d CAMERA_TO_ROBOT = new Translation2d(Units.inchesToMeters(2), CAMERA_HEIGHT);

    public static TorqueDisjointDataRegression disjointData;
    public static TorqueRollingMedian rollingMedian = new TorqueRollingMedian(10);

    public enum ShooterState implements TorqueSubsystemState {
        OFF,
        REGRESSION,
        SETPOINT,
        DISTANCE,
        WARMUP;

        public final boolean isShooting() {
            return this != OFF && this != WARMUP;
        }
    }

    private final TorqueLight camera;

    private final TorqueSparkMax hood, flywheel;

    private double hoodSetpoint, flywheelSpeed, distance, autoOffset = 0, realVelocityRPM;

    private ShooterState state = ShooterState.OFF;

    private Shooter() {
        camera = new TorqueLight("gloworm");

        // The right flywheel is set as a follower via Rev Hardware Client
        flywheel = new TorqueSparkMax(Ports.SHOOTER.FLYWHEEL.LEFT);
        flywheel.configurePID(TorquePID.create(.000005).addFeedForward(.0003725).setTolerance(20).build());

        hood = new TorqueSparkMax(Ports.SHOOTER.HOOD);
        hood.configurePID(TorquePID.create(.1)
                .addIntegral(.001)
                .addOutputRange(-.7, .7)
                .addIntegralZone(.3)
                .build());

        hood.configurePositionalCANFrame();
        hood.burnFlash();

        disjointData = new TorqueDisjointDataRegression();
        disjointData.addDisjointData(disjointData.new DisjointData(4, 20, 1800));
        disjointData.addDisjointData(disjointData.new DisjointData(5, 25, 2000));
        disjointData.addDisjointData(disjointData.new DisjointData(4.7, 25, 2000));
        disjointData.addDisjointData(disjointData.new DisjointData(3.6, 25, 1700));
        disjointData.addDisjointData(disjointData.new DisjointData(2.7, 20, 1700));
        disjointData.addDisjointData(disjointData.new DisjointData(3.8, 25, 1750));
        disjointData.addDisjointData(disjointData.new DisjointData(6.4, 35, 2500));
        disjointData.addDisjointData(disjointData.new DisjointData(5.4, 35, 2350));
        disjointData.addDisjointData(disjointData.new DisjointData(4.4, 30, 2000));
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

    public final void setAutoOffset(final double autoOffset) {
        this.autoOffset = autoOffset;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        state = ShooterState.OFF;
    }

    @Override
    public final void update(final TorqueMode mode) {
        DisjointData data = disjointData.calculate(distance);

        camera.update();

        if (climber.hasStarted()) {
            flywheelSpeed = 0;
            hoodSetpoint = HOOD_MIN;
        } else if (state == ShooterState.REGRESSION) {
            distance = getDistance();
            flywheelSpeed = data.getRPM() + (mode.isAuto() ? autoOffset : 0);
            hoodSetpoint = data.getHood();
        } else if (state == ShooterState.DISTANCE) {
            distance = getDistance();
            flywheelSpeed = data.getRPM();
            hoodSetpoint = data.getHood();
        } else if (state == ShooterState.WARMUP) {
            flywheel.setPercent(0);
        } else if (state == ShooterState.OFF) {
            flywheelSpeed = 0;
            flywheel.setPercent(0);
        }

        if (state != ShooterState.OFF) {
            flywheel.setVelocityRPM(clampRPM(flywheelSpeed));
        }

        hood.setPosition(clampHood(hoodSetpoint));

        TorqueSubsystemState.logState(state);

        SmartDashboard.putNumber("Flywheel Real", flywheel.getVelocityRPM() / FLYWHEEEL_REDUCTION);
        SmartDashboard.putNumber("Flywheel Req", flywheelSpeed);
        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putNumber("Lookup Distance", data.getDistance());

        SmartDashboard.putNumber("Flywheel Delta", flywheelSpeed - flywheel.getVelocityRPM());
        SmartDashboard.putBoolean("Is Shooting", isShooting());
        SmartDashboard.putBoolean("Is Ready", isReady());
        SmartDashboard.putNumber("Fly Wheel Current", flywheel.getCurrent());
    }

    public final boolean isShooting() {
        return state.isShooting();
    }

    public final boolean isReady() {
        return isShooting() && Math.abs(flywheelSpeed - realVelocityRPM) < ERROR;
    }

    private final double clampRPM(final double rpm) {
        return TorqueMath.constrain(rpm, FLYWHEEEL_IDLE, FLYWHEEEL_MAX);
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

    public final double getTargetOffset() {
        return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("gloworm").getEntry("targetYaw")
                .getNumber(0).doubleValue();
    }

    public static final synchronized Shooter getInstance() {
        return instance == null ? instance = new Shooter() : instance;
    }
}
