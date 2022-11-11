/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.subsystems.Elevator.ElevatorState;
import org.texastorque.subsystems.Drivebase.DrivebaseState;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

@SuppressWarnings("deprecation")
public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private Input() {
        driver = new TorqueController(0, DEADBAND);
        operator = new TorqueController(1, DEADBAND);
    }

    private boolean single = false;
    private final TorqueClick singleClick = new TorqueClick();

    @Override
    public final void update() {
        if (singleClick.calculate(driver.getBButton()))
            single = !single;

        updateDrivebase();
        updateElevator();
    }

    private final TorqueTraversableSelection<Double> translationalSpeeds = new TorqueTraversableSelection<Double>(0, 1., .75, .5, .25);

   private double invertCoefficient = 1;

    public final void invertDrivebaseControls() {
        invertCoefficient = -1;
    }

    private final PIDController rotationPID = TorquePID.create(.02 / 4).addDerivative(.001)
            .build().createPIDController((pid) -> {
                pid.enableContinuousInput(0, 360);
                return pid;
            });

    private double lastRotation = drivebase.getGyro().getRotation2d().getDegrees(), xVelo, yVelo, rVelo;

    private final TorqueSlewLimiter xLimiter = new TorqueSlewLimiter(5, 10),
            yLimiter = new TorqueSlewLimiter(5, 10);

    private final static double DEADBAND = .1;

    private final void updateDrivebase() {
        final double rotationReal = drivebase.getGyro().getRotation2d().getDegrees();
        double rotationRequested = -driver.getRightXAxis();

        SmartDashboard.putNumber("ROTREQ", rotationRequested);

        drivebase.setSpeed(translationalSpeeds.calculate(driver.getRightBumper(), driver.getLeftBumper()));

        // final boolean noInput = TorqueMath.toleranced(driver.getLeftYAxis(), DEADBAND)
        //         && TorqueMath.toleranced(driver.getLeftXAxis(), DEADBAND)
        //         && TorqueMath.toleranced(driver.getRightXAxis(), DEADBAND);

        final boolean noInput = driver.getLeftXAxis() == 0 && driver.getLeftYAxis() == 0 && driver.getRightXAxis() == 0;

        if (driver.isXButtonDown())
            drivebase.state = DrivebaseState.ZERO_WHEELS;
        // else if (driver.isYButtonDown()())
            // drivebase.state = DrivebaseState.GOTO_POS_ODOM;
        else if (noInput) drivebase.state = DrivebaseState.OFF;
        else {
            drivebase.state = DrivebaseState.DRIVING;

            if (Math.abs(rotationRequested) < .02)
                rotationRequested = -rotationPID.calculate(rotationReal, lastRotation);
            else
                lastRotation = rotationReal;
        }

        yVelo = TorqueUtil.conditionalApply(true, -driver.getLeftXAxis() * invertCoefficient,
                yLimiter::calculate);
        xVelo = TorqueUtil.conditionalApply(true, driver.getLeftYAxis() * invertCoefficient,
                xLimiter::calculate);
        rVelo = rotationRequested;

        drivebase.setSpeeds(new ChassisSpeeds(xVelo, yVelo, rVelo));
    }

    private final TorqueTraversableSelection<Double> elevatorPos = new TorqueTraversableSelection<Double>(0., 40., 96.);

    private final void updateElevator() {
        elevator.setState(ElevatorState.POSITION);
        elevatorPos.calculate(operator.isDPADDownDown() || (single && driver.isDPADDownDown()),
                operator.isDPADUpDown() || (single && driver.isDPADUpDown()));

        elevator.setLiftPos(elevatorPos.get());

        if (operator.isRightTriggerDown() || (single && driver.isRightTriggerDown()))
            elevator.setHatchDirection(TorqueDirection.FORWARD);
        else if (operator.isLeftTriggerDown() || (single && driver.isLeftTriggerDown()))
            elevator.setHatchDirection(TorqueDirection.REVERSE);
        else
            elevator.setHatchDirection(TorqueDirection.OFF);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
