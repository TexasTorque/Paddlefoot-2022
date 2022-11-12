/* Copyright 2022 Texas Torque.
**
* This file is part of Paddlefoot-2022, which is not licensed for distribution.
* For more details, see ./license.txt or write <jus@gtsbr.org>.
*/

package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueTimeout;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Elevator extends TorqueSubsystem implements Subsystems {
    private static volatile Elevator instance;

    public static final double LIFT_UP = 108., LIFT_BOTTOM = 0, LIFT_SPEED = .8;

    private double liftPos;

    private final TorqueSparkMax lift, lift2;

    private ElevatorState state = ElevatorState.OFF;

    public enum ElevatorState {
        POSITION, EXTEND, RETRACT, OFF;
    }

    public void setState(final ElevatorState state) {
        this.state = state;
    }

    private TorqueDirection liftDirection = TorqueDirection.OFF;

    private Elevator() {
        lift = new TorqueSparkMax(Ports.CLIMBER.LIFT.RIGHT);
        lift2 = new TorqueSparkMax(Ports.CLIMBER.LIFT.LEFT);

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

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putNumber("Climber Lift Position", lift.getPosition());
        SmartDashboard.putString("Lift Dir", liftDirection.toString());
        SmartDashboard.putString("Climber State", state.toString());
      
        if (state == ElevatorState.POSITION) {
            SmartDashboard.putNumber("ReqLPos", liftPos);
            lift.setPosition(TorqueMath.linearConstraint(liftPos, lift.getPosition(), LIFT_BOTTOM, LIFT_UP));
            lift2.setPosition(-TorqueMath.linearConstraint(liftPos, lift.getPosition(), LIFT_BOTTOM, LIFT_UP));

        } else if (state == ElevatorState.EXTEND) {
            lift.setPercent(LIFT_SPEED);
            lift2.setPercent(-LIFT_SPEED);
        } else if (state == ElevatorState.RETRACT) {
            lift.setPercent(-LIFT_SPEED);
            lift2.setPercent(LIFT_SPEED);
        } else {
            lift.setPercent(0);
            lift2.setPercent(0);
        }
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
