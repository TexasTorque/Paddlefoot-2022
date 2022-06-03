package org.texastorque;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Drivebase.DrivebaseState;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Intake.IntakeDirection;
import org.texastorque.subsystems.Intake.IntakePosition;
import org.texastorque.subsystems.Magazine;
import org.texastorque.subsystems.Magazine.BeltDirection;
import org.texastorque.subsystems.Magazine.GateDirection;
import org.texastorque.subsystems.Shooter;
import org.texastorque.subsystems.Turret;
import org.texastorque.subsystems.Shooter.ShooterState;
import org.texastorque.subsystems.Turret.TurretState;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.base.TorqueInputModule;
import org.texastorque.torquelib.control.complex.TorqueSpeedSettings;
import org.texastorque.torquelib.util.GenericController;

@SuppressWarnings("deprecation")
public final class Input extends TorqueInput implements Subsystems {
    private static volatile Input instance;

    private final TorqueSpeedSettings xSpeeds = new TorqueSpeedSettings(1, 0.6, 1, .2);  // 1, .8, .6
    private final TorqueSpeedSettings ySpeeds = new TorqueSpeedSettings(1, 0.6, 1, .2);  // 1, .8, .6
    private final TorqueSpeedSettings rSpeeds = new TorqueSpeedSettings(1, 0.5, 1, .25); // 1, .75, .5

    private Input() {
        driver = new GenericController(0, 0.1);
        operator = new GenericController(1, 0.1);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateIntake();
        updateMagazine();
        updateShooter();
    }

    private final void updateDrivebase() {
        drivebase.setState(DrivebaseState.FIELD_RELATIVE);
        drivebase.setSpeeds(new ChassisSpeeds(
                driver.getLeftYAxis() * Drivebase.DRIVE_MAX_TRANSLATIONAL_SPEED *
                        xSpeeds.update(driver.getRightBumper(), driver.getLeftBumper(), false, false),
                -driver.getLeftXAxis() * Drivebase.DRIVE_MAX_TRANSLATIONAL_SPEED *
                        ySpeeds.update(driver.getRightBumper(), driver.getLeftBumper(), false, false),
                -driver.getRightXAxis() * Drivebase.DRIVE_MAX_ROTATIONAL_SPEED *
                        rSpeeds.update(driver.getRightBumper(), driver.getLeftBumper(), false, false)));
    }

    private final void updateIntake() {
        if (driver.getRightTrigger()) {
            intake.setState(IntakeDirection.INTAKE, IntakePosition.DOWN);
        } else if (driver.getLeftTrigger()) {
            intake.setState(IntakeDirection.OUTAKE, IntakePosition.DOWN);
        } else {
            intake.setState(IntakeDirection.STOPPED, IntakePosition.UP);
        }
    }

    private final void updateMagazine() {
        if (operator.getRightBumper()) {
            magazine.setBeltDirection(BeltDirection.UP);
        } else if (operator.getRightTrigger()) {
            magazine.setBeltDirection(BeltDirection.DOWN);
        } else {
            magazine.setBeltDirection(BeltDirection.OFF);
        }

        if (operator.getLeftBumper()) {
            magazine.setGateDirection(GateDirection.FORWARD);
        } else if (operator.getLeftTrigger()) {
            magazine.setGateDirection(GateDirection.REVERSE);
        } else {
            magazine.setGateDirection(GateDirection.OFF);
        }
    }

    private final void updateShooter() {
        if (driver.getXButton()) {
            shooter.setState(ShooterState.REGRESSION);
            turret.setState(TurretState.TRACK);
        } else if (driver.getAButton()) {
            shooter.setState(ShooterState.MEGA_SHOT);
            turret.setState(TurretState.CENTER);
        } else if (driver.getYButton()) {
            shooter.setState(ShooterState.DISTANCE);
            shooter.setDistance(3);
            turret.setState(TurretState.CENTER);
        } else {
            shooter.setState(ShooterState.OFF);
            turret.setState(TurretState.OFF);
        }

        if (operator.getXButton()) {
            turret.setState(TurretState.POSITIONAL);
            turret.setPosition(-20);
        } else if (operator.getBButton()) {
            turret.setState(TurretState.POSITIONAL);
            turret.setPosition(20);
        }
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}