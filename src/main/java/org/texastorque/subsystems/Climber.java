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
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Climber extends TorqueSubsystem implements Subsystems {
    private static volatile Climber instance;

    public static double LIFT_UP = 97, LIFT_BOTTOM = 0, LIFT_MULTIPLIER = .8;

    // The right climber is set as a follower via Rev Hardware Client
    private final TorqueSparkMax lift, lift2;

    private ClimberState state = ClimberState.OFF;

    public enum ClimberState {
        MANUAL, EXTEND, RETRACT, OFF;
    }

    public void setState(final ClimberState state) {
        this.state = state;

    }

    private TorqueDirection liftDirection = TorqueDirection.OFF;

    public final void setManualLift(final boolean liftUp, final boolean liftDown) {
        if (liftUp)
            liftDirection = TorqueDirection.FORWARD;
        else if (liftDown)
            liftDirection = TorqueDirection.REVERSE;
        else
            liftDirection = TorqueDirection.NEUTRAL;
    }

    private Climber() {
        lift = new TorqueSparkMax(Ports.CLIMBER.LIFT.RIGHT);
        lift2 = new TorqueSparkMax(Ports.CLIMBER.LIFT.LEFT);
        lift.configurePID(TorquePID.create(.02).build());
        lift2.configurePID(TorquePID.create(.02).build());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        state = ClimberState.MANUAL;
    }

    public final void stop() {
        lift.setPercent(0);
        lift2.setPercent(0);
    }

    @Override
    public final void update(final TorqueMode mode) {

        SmartDashboard.putNumber("Climber Lift Position", lift.getPosition());
        SmartDashboard.putString("Lift Dir", liftDirection.toString());
        SmartDashboard.putString("Climber State", state.toString());

        if (state == ClimberState.MANUAL) {
            lift.setPercent(TorqueMath.linearConstraint(liftDirection.get(), lift.getPosition(), LIFT_BOTTOM, LIFT_UP) * LIFT_MULTIPLIER);
            lift2.setPercent(-TorqueMath.linearConstraint(liftDirection.get(), lift.getPosition(), LIFT_BOTTOM, LIFT_UP) * LIFT_MULTIPLIER);
        } else if (state == ClimberState.RETRACT) {
            lift.setPosition(LIFT_BOTTOM);
            lift2.setPosition(-LIFT_BOTTOM);
        } else if (state == ClimberState.EXTEND) {
            lift.setPosition(LIFT_UP);
            lift2.setPosition(-LIFT_UP);
        } else {
            lift.setPercent(0);
            lift2.setPercent(0);
        }

    }

    public final boolean hasStarted() {
        return false;
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
