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
import org.texastorque.subsystems.Intake.IntakeState;
import org.texastorque.subsystems.Magazine.MagazineState;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
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

    private final TorqueClick translate = new TorqueClick();

    private double lastRotation = drivebase.getGyro().getRotation2d().getDegrees(), xVelo, yVelo, rVelo;

    private final TorqueSlewLimiter xLimiter = new TorqueSlewLimiter(5, 10),
            yLimiter = new TorqueSlewLimiter(5, 10);

    private final static double DEADBAND = .04;

    private final void updateDrivebase() {
        final double rotationReal = drivebase.getGyro().getRotation2d().getDegrees();
        double rotationRequested = -driver.getRightXAxis();

        drivebase.setSpeed(translationalSpeeds.calculate(driver.getRightBumper(), driver.getLeftBumper()));

        final boolean noInput = TorqueMath.toleranced(driver.getLeftYAxis(), DEADBAND)
                && TorqueMath.toleranced(driver.getLeftXAxis(), DEADBAND)
                && TorqueMath.toleranced(driver.getRightXAxis(), DEADBAND);
                //&& !driver.getYButton(); // this shouldnt matter

        // TEST FUNCTIONALITY
        if (translate.calculate(driver.getYButton())) {
            // final var trans = new Translation2d(4, 4);
            // drivebase.setDesiredPosition(new Pose2d(drivebase.getPose().getTranslation().plus(trans),
            //         Rotation2d.fromDegrees(90)));
        }

        if (driver.getLeftCenterButton())
            drivebase.state = DrivebaseState.ZERO_WHEELS;
        else if (driver.getYButton())
            drivebase.state = DrivebaseState.GOTO_POS_ODOM;
        else if (noInput) drivebase.state = DrivebaseState.OFF;
        else {
            drivebase.state = DrivebaseState.DRIVING;

            if (rotationRequested == 0)
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

        SmartDashboard.putNumber("X Velo", xVelo);
        SmartDashboard.putNumber("Y Velo", yVelo);
        SmartDashboard.putNumber("R Velo", rVelo);
        SmartDashboard.putNumber("PID O", rotationRequested);
        SmartDashboard.putNumber("Rot Delta", rotationReal - lastRotation);
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
        if (driver.getAButton())
            magazine.setState(MagazineState.OUT);
        else if (driver.getLeftTrigger())
            magazine.setState(MagazineState.POP);
        else
            magazine.setState(MagazineState.OFF);
    }


    private final TorqueTraversableSelection<Double> elevatorPos = new TorqueTraversableSelection<Double>(0., 40., 96.);

    private final void updateElevator() {
        elevator.setState(ElevatorState.POSITION);
        elevatorPos.calculate(operator.getDPADDown() || driver.getDPADDown(),
                operator.getDPADUp() || driver.getDPADUp());

        elevator.setLiftPos(elevatorPos.get());

        if (operator.getRightTrigger())
            elevator.setHatchDirection(TorqueDirection.FORWARD);
        else if (operator.getLeftTrigger())
            elevator.setHatchDirection(TorqueDirection.REVERSE);
        else
            elevator.setHatchDirection(TorqueDirection.OFF);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
