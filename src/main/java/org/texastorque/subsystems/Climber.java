/**
 * Copyright 2022 Texas Torque.
 * 
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


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Climber extends TorqueSubsystem implements Subsystems {
    private static volatile Climber instance;

    public static double LIFT_UP = .1477, HOOK_OPEN = .1477, LIFT_BOTTOM = .1477, LIFT_MULTIPLIER = 1, HOOK_MULTIPLIER = 1; 
    // TODO(@Juicestus <jus@justusl.com>): FILL THESE

    private final TorqueSparkMax lift1, lift2; 
    private final TorqueSparkMax hook; 

    private ClimberState state = ClimberState.MANUAL;

    private boolean clamped = false;

    public enum ClimberState {
        MANUAL, READY, PULL, OFF;
    }

    public void setState(final ClimberState state) { 
        this.state = state;
    
    }
    private TorqueDirection liftDirection = TorqueDirection.OFF;
    private TorqueDirection hookDirection = TorqueDirection.OFF;

    public final void setManualLift(final boolean liftUp, final boolean liftDown) {
        if (liftUp)
            liftDirection = TorqueDirection.FORWARD;
        else if (liftDown)
            liftDirection = TorqueDirection.REVERSE;
        else
            liftDirection = TorqueDirection.NEUTRAL;
    }

    public final void setManualHook(final boolean hookUp, final boolean hookDown) {
        if (hookUp)
            hookDirection = TorqueDirection.FORWARD;
        else if (hookDown)
            hookDirection = TorqueDirection.REVERSE;
        else
            hookDirection = TorqueDirection.NEUTRAL;
    }

    private Climber() {
        lift1 = new TorqueSparkMax(Ports.CLIMBER.LIFT.RIGHT);
        lift2 = new TorqueSparkMax(Ports.CLIMBER.LIFT.LEFT);
        // lift.addFollower(Ports.CLIMBER.LIFT.LEFT, false);

        // lift.configurePID(TorquePID.create(.1).build());

        hook = new TorqueSparkMax(Ports.CLIMBER.HOOK);
        hook.configurePID(TorquePID.create(.1).build());
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        state = ClimberState.MANUAL;
        clamped = false;
        // stop();
    }

    public final void stop() {
        // lift.setPercent(0);
        if (!clamped)
            hook.setPercent(0);
    }

    @Override
    public final void update(final TorqueMode mode) {

        SmartDashboard.putNumber("Climber Lift 1", lift1.getPosition());
        SmartDashboard.putNumber("Climber Lift 2", lift2.getPosition());
        SmartDashboard.putNumber("Climber Hook", hook.getPosition());
        SmartDashboard.putString("Lift Dir", liftDirection.toString());
        SmartDashboard.putString("Hook Dir", hookDirection.toString());

        if (state == ClimberState.MANUAL) {
            lift1.setPercent(liftDirection.get() * LIFT_MULTIPLIER);
            lift2.setPercent(liftDirection.get() * LIFT_MULTIPLIER);
            hook.setPercent(hookDirection.get() * HOOK_MULTIPLIER);
        } else {
            lift1.setPercent(0);
            lift2.setPercent(0);
            // hook.setPercent(0);
        }

        // if (!mode.isTeleop()) {
        //     stop();
        //     return;
        // }

        // if (state == ClimberState.MANUAL) {
        //     lift.setPercent(liftDirection.get());
        //     hook.setPercent(hookDirection.get());
        //     return;
        // }





        // if (state == ClimberState.READY) {
        //     lift.setPosition(LIFT_UP);
        //     hook.setPosition(HOOK_OPEN);
        // }

        // if (state == ClimberState.PULL) {
        //     clamped = true;

        //     // if (lift.getPosition() <= LIFT_BOTTOM)
        //     //     lift.setPercent(TorqueDirection.OFF.get());
        //     // else
        //     //     lift.setPercent(TorqueDirection.REVERSE.get());

        //     lift.setPercent((lift.getPosition() <= LIFT_BOTTOM ? TorqueDirection.OFF : TorqueDirection.REVERSE).get());
        //     hook.setVoltage(TorqueDirection.FORWARD.get() * 12);

        // }
    }

    public final void setClamped(final boolean clamped) {
        this.clamped = clamped;
    }

    public final boolean hasStarted() { return false; }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
