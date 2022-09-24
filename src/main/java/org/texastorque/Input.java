/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.subsystems.Shooter;
import org.texastorque.subsystems.Climber.ClimberState;
import org.texastorque.subsystems.Intake.IntakeState;
import org.texastorque.subsystems.Magazine;
import org.texastorque.subsystems.Shooter.ShooterState;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueTraversableRange;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

@SuppressWarnings("deprecation")
public final class Input extends TorqueInput<GenericController> implements Subsystems {
    private static volatile Input instance;

    private Input() {
        driver = new GenericController(0, .1);
        operator = new GenericController(1, .1);
    }

    @Override
    public final void update() {
        updateDrivebase();
        updateIntake();
        updateMagazine();
        updateShooter();
        updateClimber();
    }

    private final TorqueTraversableSelection<Double> translationalSpeeds = new TorqueTraversableSelection<Double>(1, .5,
            .6, .7),
            rotationalSpeeds = new TorqueTraversableSelection<Double>(1, .5, .75, 1.);
    // Incredibly basic solution for inverting the driver controls after an auto routine.
    private double invertCoefficient = 1;

    public final void invertDrivebaseControls() {
        invertCoefficient = -1;
    }

    // private final PIDController rotationPID = TorquePID.create(.02 / 4).addDerivative(.001)
    //         .build().createPIDController((pid) -> {
    //             pid.enableContinuousInput(0, 360);
    //             return pid;
    //         });

    private double lastRotation = drivebase.getGyro().getRotation2d().getDegrees(), xVelo, yVelo, rVelo;

    private final TorqueSlewLimiter xLimiter = new TorqueSlewLimiter(5, 10),
            yLimiter = new TorqueSlewLimiter(5, 10);

    private final static double DEADBAND = .04;

    private final void updateDrivebase() {
        SmartDashboard.putNumber("Speed Shifter", (rotationalSpeeds.get() - .5) * 2.);

        final double rotationReal = drivebase.getGyro().getRotation2d().getDegrees();
        double rotationRequested = -driver.getRightXAxis();

        // if (rotationRequested == 0)
        //     rotationRequested = -rotationPID.calculate(rotationReal, lastRotation);
        // else
        //     lastRotation = rotationReal;

        SmartDashboard.putNumber("PID O", rotationRequested);
        SmartDashboard.putNumber("Rot Delta", rotationReal - lastRotation);

        drivebase.setSpeedCoefs(translationalSpeeds.calculate(driver.getLeftBumper(), driver.getRightBumper()),
                rotationalSpeeds.calculate(driver.getLeftBumper(), driver.getRightBumper()));

        drivebase.setSpeedCoefs(1, 1);

        final boolean noInput = TorqueMath.toleranced(driver.getLeftYAxis(), DEADBAND)
                && TorqueMath.toleranced(driver.getLeftXAxis(), DEADBAND)
                && TorqueMath.toleranced(driver.getRightXAxis(), DEADBAND);

        if (noInput) {
            drivebase.setSpeeds(new ChassisSpeeds(0, 0, 0));
            return;
        }

        xVelo = TorqueUtil.conditionalApply(true, driver.getLeftYAxis() * invertCoefficient,
                xLimiter::calculate);
        yVelo = TorqueUtil.conditionalApply(true, -driver.getLeftXAxis() * invertCoefficient,
                yLimiter::calculate);
        rVelo = rotationRequested;
        drivebase.setSpeeds(new ChassisSpeeds(xVelo, yVelo, rVelo));
        
        SmartDashboard.putNumber("X Velo", xVelo);
        SmartDashboard.putNumber("Y Velo", yVelo);
        SmartDashboard.putNumber("R Velo", rVelo);
    }

    private final void updateIntake() {

        if (driver.getRightTrigger())
            intake.setState(IntakeState.INTAKE);
        else if (driver.getAButton())
            intake.setState(IntakeState.OUTAKE);
        else
            intake.setState(IntakeState.PRIMED);
    }

    private final void updateMagazine() {
        updateManualMagazineBeltControls(operator);
        updateManualMagazineGateControls(operator);
    }

    private final void updateManualMagazineBeltControls(final GenericController controller) {
        if (controller.getRightTrigger())
            magazine.setBeltDirection(Magazine.MAG_UP);
        else if (controller.getLeftTrigger())
            magazine.setBeltDirection(Magazine.MAG_DOWN);
        else
            magazine.setBeltDirection(TorqueDirection.OFF);
    }

    private final void updateManualMagazineGateControls(final GenericController controller) {
        if (controller.getRightBumper())
            magazine.setGateDirection(TorqueDirection.FORWARD);
        else if (controller.getLeftBumper())
            magazine.setGateDirection(TorqueDirection.REVERSE);
        else
            magazine.setGateDirection(TorqueDirection.OFF);
    }

    private final TorqueTraversableRange flywheelRPM = new TorqueTraversableRange(1000, 200, 4000, 50);
    private final TorqueTraversableRange hoodSetpoint = new TorqueTraversableRange(Shooter.HOOD_MIN, Shooter.HOOD_MIN,
            Shooter.HOOD_MAX, 5);

    private final void updateShooter() {
        flywheelRPM.update(operator.getDPADRight(), operator.getDPADLeft(), false, false);
        hoodSetpoint.update(operator.getYButton(), operator.getAButton(), false, false);

        SmartDashboard.putNumber("IRPM", flywheelRPM.getSpeed());
        SmartDashboard.putNumber("IHOOD", hoodSetpoint.getSpeed());

        // This is debugging for the regression
        if (operator.getXButton()) {
            shooter.setState(ShooterState.SETPOINT);
            shooter.setFlywheelSpeed(flywheelRPM.getSpeed());
            shooter.setHoodPosition(hoodSetpoint.getSpeed());
        } else if (driver.getLeftTrigger()) {
            shooter.setState(ShooterState.REGRESSION);
        } else if (driver.getYButton()) {
            shooter.setState(ShooterState.SETPOINT);
            shooter.setFlywheelSpeed(1700);
            shooter.setHoodPosition(30);
        } else if (driver.getXButton()) {
            shooter.setState(ShooterState.SETPOINT);
            shooter.setFlywheelSpeed(1200);
            shooter.setHoodPosition(25);
        } else {
            shooter.setState(ShooterState.OFF);
            //shooter.setState(ShooterState.IDLE);
        }
    }

    private final void updateClimber() {
        if (operator.getYButton() || operator.getAButton()) {
            climber.setState(ClimberState.MANUAL);
            climber.setManualLift(operator.getYButton(), operator.getAButton());
        } else
            climber.setState(ClimberState.OFF);

    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
