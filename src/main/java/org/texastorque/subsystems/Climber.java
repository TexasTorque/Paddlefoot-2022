/* Copyright 2022 Texas Torque.
**
* This file is part of Clutch-2022, which is not licensed for distribution.
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

    public static double LIFT_UP = .1477, HOOK_OPEN = .1477, LIFT_BOTTOM = .1477;
    // TODO(@Juicestus <jus@justusl.com>): FILL THESE

    // The right climber is set as a follower via Rev Hardware Client
    private final TorqueSparkMax lift;

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
        lift = new TorqueSparkMax(Ports.CLIMBER.LIFT.LEFT);
        lift.configurePID(TorquePID.create(.007).build());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        state = ClimberState.MANUAL;
    }

    public final void stop() {
        lift.setPercent(0);
    }

    @Override
    public final void update(final TorqueMode mode) {

        SmartDashboard.putNumber("Climber Lift Position", lift.getPosition());
        SmartDashboard.putString("Lift Dir", liftDirection.toString());

        if (state == ClimberState.MANUAL) {
            lift.setPercent(TorqueMath.linearConstraint(liftDirection.get(), lift.getPosition(), LIFT_BOTTOM, LIFT_UP));
        } else if (state == ClimberState.RETRACT) {
            lift.setPosition(LIFT_BOTTOM);
        } else if (state == ClimberState.EXTEND) {
            lift.setPosition(LIFT_UP);
        } else {
            lift.setPercent(0);
        }

    }

    public final boolean hasStarted() {
        return false;
    }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
