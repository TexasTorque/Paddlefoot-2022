/* Copyright 2022 Texas Torque.
**
* This file is part of Paddlefoot-2022, which is not licensed for distribution.
* For more details, see ./license.txt or write <jus@gtsbr.org>.
*/

package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Elevator extends TorqueSubsystem implements Subsystems {
    private static volatile Elevator instance;

    public static double LIFT_UP = 108, LIFT_BOTTOM = 0, LIFT_MULTIPLIER = .8, liftPos;

    private final TorqueSparkMax lift, lift2, hatch;

    private ElevatorState state = ElevatorState.OFF;

    public enum ElevatorState {
        POSITION, EXTEND, RETRACT, OFF;
    }

    public void setState(final ElevatorState state) {
        this.state = state;

    }

    private TorqueDirection liftDirection = TorqueDirection.OFF, hatchDirection = TorqueDirection.OFF;

    private Elevator() {
        lift = new TorqueSparkMax(Ports.CLIMBER.LIFT.RIGHT);
        lift2 = new TorqueSparkMax(Ports.CLIMBER.LIFT.LEFT);
        hatch = new TorqueSparkMax(Ports.CLIMBER.HATCH);

        lift.configurePID(TorquePID.create(.2).build());
        lift2.configurePID(TorquePID.create(.2).build());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    // Designed for a lift and retract button
    public final void setManualLift(final boolean liftUp, final boolean liftDown) {
        if (liftUp)
            liftDirection = TorqueDirection.FORWARD;
        else if (liftDown)
            liftDirection = TorqueDirection.REVERSE;
        else
            liftDirection = TorqueDirection.NEUTRAL;

    }

    public void setLiftPos(double position) {
        liftPos = position;
    }

    public void setHatchDirection(TorqueDirection direction) {
        hatchDirection = direction;
    }

    private int spikeLevel = 0;
    private final TorqueClick spikeClick = new TorqueClick();
    private boolean hasHatch = false;

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putNumber("Climber Lift Position", lift.getPosition());
        SmartDashboard.putString("Lift Dir", liftDirection.toString());
        SmartDashboard.putString("Climber State", state.toString());
        SmartDashboard.putNumber("Hatch", hatchDirection.get());

        SmartDashboard.putNumber("Current", hatch.getCurrent());

        SmartDashboard.putNumber("Spike", spikeLevel);

        if (hatchDirection == TorqueDirection.FORWARD) {
            if (spikeClick.calculate(hatch.getCurrent() >= 12))
                spikeLevel++;
        } else 
            spikeLevel = 0;

        if (hatchDirection == TorqueDirection.REVERSE)
            hasHatch = false;

        if (spikeLevel >= 2 || hasHatch) {
            hasHatch = true;
            hatch.setPercent(0);
        }  else 
            hatch.setPercent(hatchDirection.get());


        if (state == ElevatorState.POSITION) {
            lift.setPosition(TorqueMath.linearConstraint(liftPos, lift.getPosition(), LIFT_BOTTOM, LIFT_UP)
                    * LIFT_MULTIPLIER);
            lift2.setPosition(-TorqueMath.linearConstraint(liftPos, lift.getPosition(), LIFT_BOTTOM, LIFT_UP)
                    * LIFT_MULTIPLIER);
        } else if (state == ElevatorState.EXTEND) {
            lift.setPercent(LIFT_MULTIPLIER);
            lift2.setPercent(-LIFT_MULTIPLIER);
        } else if (state == ElevatorState.RETRACT) {
            lift.setPercent(-LIFT_MULTIPLIER);
            lift2.setPercent(LIFT_MULTIPLIER);
        } else {
            lift.setPercent(0);
            lift2.setPercent(0);
        }

        hatchDirection = TorqueDirection.OFF;
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
