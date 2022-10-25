/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueTimeout;
import org.texastorque.torquelib.modules.TorqueSwerveModule2021;
import org.texastorque.torquelib.sensors.TorqueLight;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.sensors.util.TorqueAprilTagMap;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueSwerveOdometry;
import org.texastorque.torquelib.util.TorqueUtil;

/**
 * The drivebase subsystem. Drives with 4 2021 swerve modules.
 *
 * @author Jack Pittenger
 * @author Justus Languell
 * @author Omar Afzal
 */
@SuppressWarnings("deprecation")
public final class Drivebase extends TorqueSubsystem implements Subsystems {
    private static volatile Drivebase instance;

    public static final double DRIVE_MAX_TRANSLATIONAL_SPEED = 4, DRIVE_MAX_TRANSLATIONAL_ACCELERATION = 4, // TEST NEW MAX SPEEDS
            DRIVE_MAX_ROTATIONAL_SPEED = 4 * Math.PI;

    // These are constants for our specifics swerve modules
    private static final double DRIVE_GEARING = .1875, // Drive rotations per motor rotations
            DRIVE_WHEEL_RADIUS = Units.inchesToMeters(1.788), DISTANCE_TO_CENTER_X = Units.inchesToMeters(10.875),
            DISTANCE_TO_CENTER_Y = Units.inchesToMeters(10.875);
    public static final TorquePID DRIVE_PID = TorquePID.create(.00048464).addIntegralZone(.2).build();
    public static final TorquePID ROTATE_PID = TorquePID.create(.3).build();
    public final SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(.27024, 2.4076, .5153);

    // These PID controllers are for the macro position control during targeting sequences
    public final ProfiledPIDController thetaController;
    private final HolonomicDriveController controller;
    private final TorquePID xController = TorquePID.create(8).build();
    private final TorquePID yController = TorquePID.create(8).build();

    // This caches the desired position translation
    private Pose2d desired = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    // These constants are translation representations of the locations of the swerve modules
    private final Translation2d 
            locationBackLeft = new Translation2d(DISTANCE_TO_CENTER_X, -DISTANCE_TO_CENTER_Y),
            locationBackRight = new Translation2d(DISTANCE_TO_CENTER_X, DISTANCE_TO_CENTER_Y),
            locationFrontLeft = new Translation2d(-DISTANCE_TO_CENTER_X, -DISTANCE_TO_CENTER_Y),
            locationFrontRight = new Translation2d(-DISTANCE_TO_CENTER_X, DISTANCE_TO_CENTER_Y);

    // This is the kinematics object that calculates the desired wheel speeds
    private final SwerveDriveKinematics kinematics;

    // private final TorqueSwerveOdometry odometry;

    // PoseEstimator is a more advanced odometry system that uses a Kalman filter to estimate the robot's position
    // It also encorporates other measures like April tag positions
    private final SwerveDrivePoseEstimator poseEstimator;
    
    // Matrix constants for the pose estimator.
    private static final Matrix<N3, N1> STATE_STDS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, .02);
    private static final Matrix<N1, N1> LOCAL_STDS = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(.01);
    private static final Matrix<N3, N1> VISION_STDS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(.025, .025, .025);

    // The swerve module objects
    private final TorqueSwerveModule2021 backLeft, backRight, frontLeft, frontRight;
    // The states to be mapped
    private SwerveModuleState[] swerveModuleStates;
    // The cached vector of the robot
    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    // The instance of the NavX
    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();
    // Coef for the speed shifter
    private double speed = 1;
    // The field representation of the robot for logging
    private final Field2d aprilField = new Field2d();
    // A table of April tags by ID and their positions
    private final Map<Integer, Pose3d> aprilTags;

    // The TorqueLight object
    private final TorqueLight camera;
    public final TorqueLight getCamera() { return camera; }

    // The state of the drivebase (no shit)
    public enum DrivebaseState {
        OFF,
        DRIVING,
        GOTO_POS_ODOM,
        ZERO_WHEELS
    } 
    public DrivebaseState state;

    private Drivebase() {
        state = DrivebaseState.OFF;

        // Instantiating the individual modules
        backLeft = buildSwerveModule(0, Ports.DRIVEBASE.TRANSLATIONAL.LEFT.BACK, Ports.DRIVEBASE.ROTATIONAL.LEFT.BACK);
        backRight = buildSwerveModule(1, Ports.DRIVEBASE.TRANSLATIONAL.RIGHT.BACK,
                Ports.DRIVEBASE.ROTATIONAL.RIGHT.BACK);
        frontLeft = buildSwerveModule(2, Ports.DRIVEBASE.TRANSLATIONAL.LEFT.FRONT,
                Ports.DRIVEBASE.ROTATIONAL.LEFT.FRONT);
        frontRight = buildSwerveModule(3, Ports.DRIVEBASE.TRANSLATIONAL.RIGHT.FRONT,
                Ports.DRIVEBASE.ROTATIONAL.RIGHT.FRONT);

        // Instantiating the kinematics object
        kinematics = new SwerveDriveKinematics(locationBackLeft, locationBackRight, locationFrontLeft,
                locationFrontRight);

        // odometry = new TorqueSwerveOdometry(kinematics, gyro.getRotation2dClockwise());

        // Instantiating the pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(gyro.getRotation2dClockwise().times(-1),
                new Pose2d(), kinematics, STATE_STDS,
                LOCAL_STDS, VISION_STDS);

        // Instantiating and configuring the positional PID controllers
        thetaController = new ProfiledPIDController(4, 0, 0,
                new TrapezoidProfile.Constraints(3 * Math.PI, 3 * Math.PI));
        thetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        controller = new HolonomicDriveController(xController, yController, thetaController);

        // Putting the aprilField object to Shuffleboard
        SmartDashboard.putData("April Position", aprilField);

        aprilTags = new HashMap<Integer, Pose3d>();
        aprilTags.put(0, new Pose3d(6.6, 3.4, 1.6002, new Rotation3d(0, 0, 15)));
        aprilTags.put(69, new Pose3d(9, 2.45, 1.6002, new Rotation3d(0, 0, 105)));
        //aprilTags = TorqueAprilTagMap.fromJSON();

        camera = new TorqueLight();

        stopMoving();
    }

    public final void setSpeed(final double speed) {
        this.speed = speed;
    }

    public final void setSpeeds(final ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public final ChassisSpeeds getSpeeds() {
        return speeds;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        stopMoving();
    }

    public final void updateFeedback() {
        poseEstimator.update(gyro.getRotation2dClockwise().times(-1),
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
        // field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        if (camera.hasTargets()) {
            final Pose2d pose =  camera.getRobotPoseAprilTag(aprilTags, CAMERA_ANGLE.getDegrees(), CAMERA_HEIGHT, 
                    gyro.getRotation2dCounterClockwise(), 90);
            SmartDashboard.putString("ApilPos", String.format("(%02.3f, %02.3f)", pose.getX(), pose.getY()));
            aprilField.setRobotPose(pose);
            poseEstimator.addVisionMeasurement(pose, TorqueUtil.time() - camera.getLatency() / 1000.);
        }
    }

    private static final Rotation2d CAMERA_ANGLE = new Rotation2d(0); // from which way?
    private static final double CAMERA_HEIGHT = Units.inchesToMeters(33); 

    private final TorqueClick creepClick = new TorqueClick();
    private final TorqueTimeout creepTimeout = new TorqueTimeout(0.4);

    @Override
    public final void update(final TorqueMode mode) {
        camera.update();

        // odometry.update(gyro.getRotation2dClockwise().times(-1), frontLeft.getState(), frontRight.getState(),
        //         backLeft.getState(), backRight.getState());
        updateFeedback();

        if (state == DrivebaseState.ZERO_WHEELS) {
            zeroWheels();
        } else {
            if (creepTimeout.calculate(creepClick.calculate(elevator.isOutaking())))
                elevatorCreep();
            else if (state == DrivebaseState.DRIVING)
                normalDriving(mode);
            else if (state == DrivebaseState.GOTO_POS_ODOM)
                alignToTag();
            else
                stopMoving();

            swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                    DRIVE_MAX_TRANSLATIONAL_SPEED * speed);

            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            backLeft.setDesiredState(swerveModuleStates[2]);
            backRight.setDesiredState(swerveModuleStates[3]);
        }

        log();
    }

    public void elevatorCreep() {
        speeds = new ChassisSpeeds(0, -2, 0);
    }

    public final void normalDriving(final TorqueMode mode) {
        if (mode.isTeleop())
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond * DRIVE_MAX_TRANSLATIONAL_SPEED,
                    speeds.vyMetersPerSecond * DRIVE_MAX_TRANSLATIONAL_SPEED,
                    speeds.omegaRadiansPerSecond * DRIVE_MAX_ROTATIONAL_SPEED,
                    gyro.getRotation2dClockwise());

    }

    public final void alignToTag() {

        final ChassisSpeeds adjustedSpeeds = controller.calculate(
                getPose(), desired, 0,
                Rotation2d.fromDegrees(0));
        adjustedSpeeds.vxMetersPerSecond *= -1;
        speeds = adjustedSpeeds;
    }

    public void zeroWheels() {
        frontLeft.setRotatePosition(0);
        frontRight.setRotatePosition(0);
        backLeft.setRotatePosition(0);
        backRight.setRotatePosition(0);
    }

    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // public final TorqueSwerveOdometry getOdometry() {
    //     return odometry;
    // }

    public final Pose2d getPose() {
        // return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
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

    public final void stopMoving() {
        speeds = new ChassisSpeeds(0, 0, 0);
    }

    private final TorqueSwerveModule2021 buildSwerveModule(final int id, final int drivePort, final int rotatePort) {
        return new TorqueSwerveModule2021(id, drivePort, rotatePort, DRIVE_GEARING, DRIVE_WHEEL_RADIUS, DRIVE_PID,
                ROTATE_PID, DRIVE_MAX_TRANSLATIONAL_SPEED,
                DRIVE_MAX_TRANSLATIONAL_ACCELERATION, DRIVE_FEED_FORWARD);
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