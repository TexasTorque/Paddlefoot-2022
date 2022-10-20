/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.modules.TorqueSwerveModule2021;
import org.texastorque.torquelib.sensors.TorqueLight;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueSwerveOdometry;

/**
 * The drivebase subsystem. Drives with 4 2021 swerve modules.
 *
 * @author Jack Pittenger
 * @author Justus Languell
 */
@SuppressWarnings("deprecation")
public final class Drivebase extends TorqueSubsystem implements Subsystems {
    private static volatile Drivebase instance;

    public static final double DRIVE_MAX_TRANSLATIONAL_SPEED = 25, DRIVE_MAX_TRANSLATIONAL_ACCELERATION = 25, // TEST NEW MAX SPEEDS
            DRIVE_MAX_ROTATIONAL_SPEED = 25 * Math.PI, TOLERANCE = 7, ROCKET_X_OFFSET = 5, ROCKET_Y_OFFSET = 5;

    private static final double DRIVE_GEARING = .1875, // Drive rotations per motor rotations
            DRIVE_WHEEL_RADIUS = Units.inchesToMeters(1.788), DISTANCE_TO_CENTER_X = Units.inchesToMeters(10.875),
            DISTANCE_TO_CENTER_Y = Units.inchesToMeters(10.875);

    public static final TorquePID DRIVE_PID = TorquePID.create(.00048464).addIntegralZone(.2).build();
    public static final TorquePID ROTATE_PID = TorquePID.create(.3).build();

    public final SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(.27024, 2.4076, .5153);

    public final ProfiledPIDController thetaController;
    private final HolonomicDriveController controller;

    private final TorquePID xController = TorquePID.create(8).build();
    private final TorquePID yController = TorquePID.create(8).build();

    private Pose2d desired = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    private final Translation2d locationBackLeft = new Translation2d(DISTANCE_TO_CENTER_X, -DISTANCE_TO_CENTER_Y),
            locationBackRight = new Translation2d(DISTANCE_TO_CENTER_X, DISTANCE_TO_CENTER_Y),
            locationFrontLeft = new Translation2d(-DISTANCE_TO_CENTER_X, -DISTANCE_TO_CENTER_Y),
            locationFrontRight = new Translation2d(-DISTANCE_TO_CENTER_X, DISTANCE_TO_CENTER_Y);

    private final SwerveDriveKinematics kinematics;
    private final TorqueSwerveOdometry odometry;

    private final TorqueSwerveModule2021 backLeft, backRight, frontLeft, frontRight;
    private SwerveModuleState[] swerveModuleStates;

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private double translationalSpeedCoef, rotationalSpeedCoef, offset;

    private boolean shouldTarget = false;

    private final TorquePID targetPID = TorquePID.create(.6).build();;

    public DrivebaseState state;

    public enum DrivebaseState {
        OFF,
        DRIVING,
        ALIGN_TO_TAG,
        ZERO_WHEELS
    }

    public void setState(DrivebaseState state) {
        this.state = state;
    }

    public DrivebaseState getState() {
        return state;
    }

    private Drivebase() {
        state = DrivebaseState.OFF;
        backLeft = buildSwerveModule(0, Ports.DRIVEBASE.TRANSLATIONAL.LEFT.BACK, Ports.DRIVEBASE.ROTATIONAL.LEFT.BACK);
        backLeft.setLogging(true);
        backRight = buildSwerveModule(1, Ports.DRIVEBASE.TRANSLATIONAL.RIGHT.BACK,
                Ports.DRIVEBASE.ROTATIONAL.RIGHT.BACK);
        frontLeft = buildSwerveModule(2, Ports.DRIVEBASE.TRANSLATIONAL.LEFT.FRONT,
                Ports.DRIVEBASE.ROTATIONAL.LEFT.FRONT);
        frontRight = buildSwerveModule(3, Ports.DRIVEBASE.TRANSLATIONAL.RIGHT.FRONT,
                Ports.DRIVEBASE.ROTATIONAL.RIGHT.FRONT);

        kinematics = new SwerveDriveKinematics(locationBackLeft, locationBackRight, locationFrontLeft,
                locationFrontRight);

        odometry = new TorqueSwerveOdometry(kinematics, gyro.getRotation2dClockwise());

        thetaController = new ProfiledPIDController(4, 0, 0,
                new TrapezoidProfile.Constraints(3 * Math.PI, 3 * Math.PI));
        thetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        controller = new HolonomicDriveController(xController, yController, thetaController);

        reset();
    }

    public final void setSpeedCoefs(final double translational, final double rotational) {
        this.translationalSpeedCoef = translational;
        this.rotationalSpeedCoef = rotational;
    }

    public final void setSpeeds(final ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public final ChassisSpeeds getSpeeds() {
        return speeds;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        if (mode.isTeleop())
            shouldTarget = true;

        reset();
    }

    @Override
    public final void update(final TorqueMode mode) {

        odometry.update(gyro.getRotation2dClockwise().times(-1), frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());

        if (state == DrivebaseState.ZERO_WHEELS) {
            zeroWheels();
        } else if (state == DrivebaseState.OFF) {
            // Do nothing!
        } else {
            if (state == DrivebaseState.DRIVING)
                normalDriving(mode);
            else if (state == DrivebaseState.ALIGN_TO_TAG)
                alignToTag();

            swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                    DRIVE_MAX_TRANSLATIONAL_SPEED);

            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            backLeft.setDesiredState(swerveModuleStates[2]);
            backRight.setDesiredState(swerveModuleStates[3]);
        }

        log();
    }

    public final void normalDriving(final TorqueMode mode) {
        final double maxTranslatingSpeed = translationalSpeedCoef * DRIVE_MAX_TRANSLATIONAL_SPEED;
        final double maxRotationalSpeed = rotationalSpeedCoef * DRIVE_MAX_ROTATIONAL_SPEED;

        if (mode.isTeleop())
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond * maxTranslatingSpeed,
                    speeds.vyMetersPerSecond * maxTranslatingSpeed,
                    speeds.omegaRadiansPerSecond * maxRotationalSpeed,
                    gyro.getRotation2dClockwise());

    }

    public final void alignToTag() {
        final ChassisSpeeds adjustedSpeeds = controller.calculate(
                getOdometry().getPoseMeters(), desired, 0,
                Rotation2d.fromDegrees(0));
        adjustedSpeeds.vxMetersPerSecond *= -1;
        speeds = adjustedSpeeds;
    }

    // public ChassisSpeeds alignWithVision() {
    //     double xToRocket = magazine.getDistance() / Math.asin(shooter.getTargetOffset()) + ROCKET_X_OFFSET;
    //     double YToRocket = shooter.getDistance() / Math.acos(shooter.getTargetOffset()) + ROCKET_Y_OFFSET;

    //     ChassisSpeeds adjustedSpeeds = controller.calculate(
    //             getOdometry().getPoseMeters(), new Pose2d(new Translation2d(xToRocket, YToRocket), new Rotation2d()), 0,
    //             Rotation2d.fromDegrees(0));
    //     return adjustedSpeeds;
    // }

    public void zeroWheels() {
        frontLeft.setRotatePosition(0);
        frontRight.setRotatePosition(0);
        backLeft.setRotatePosition(0);
        backRight.setRotatePosition(0);
    }

    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public final TorqueSwerveOdometry getOdometry() {
        return odometry;
    }

    public final Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public final TorqueNavXGyro getGyro() {
        return gyro;
    }

    public final void log() {
        SmartDashboard.putString("OdomPos", String.format("(%02.3f, %02.3f)", getPose().getX(), getPose().getY()));

        SmartDashboard.putNumber("Odom Rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Rot", gyro.getRotation2dClockwise().getDegrees());
        SmartDashboard.putNumber("Gyro Rot -1", gyro.getRotation2dClockwise().times(-1).getDegrees());

        SmartDashboard.putString("Speeds", String.format("(%02.3f, %02.3f, %02.3f)", speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));

        SmartDashboard.putString("Drivebase State", state.toString());

        SmartDashboard.putNumber("Desired X", desired.getX());
        SmartDashboard.putNumber("Desired Y", desired.getY());
        SmartDashboard.putNumber("Desired Rotation", desired.getRotation().getDegrees());
    }

    public final void reset() {
        speeds = new ChassisSpeeds(0, 0, 0);
    }

    private final TorqueSwerveModule2021 buildSwerveModule(final int id, final int drivePort, final int rotatePort) {
        return new TorqueSwerveModule2021(id, drivePort, rotatePort, DRIVE_GEARING, DRIVE_WHEEL_RADIUS, DRIVE_PID,
                ROTATE_PID, DRIVE_MAX_TRANSLATIONAL_SPEED,
                DRIVE_MAX_TRANSLATIONAL_ACCELERATION, DRIVE_FEED_FORWARD);
    }

    public final double getDisplacement() {
        return frontLeft.getDisplacement();
    }

    public void setDesiredPosition(final Pose2d desired) {
        this.desired = desired;
    }

    public Pose2d getDesired() {
        return desired;
    }

    public static final synchronized Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}