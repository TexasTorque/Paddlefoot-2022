package org.texastorque.auto.commands;

/**
* Copyright 2022 Texas Torque.
*
* This file is part of Clutch-2022, which is not licensed for distribution.
* For more details, see ./license.txt or write <jus@gtsbr.org>.
*/

import edu.wpi.first.wpilibj.Timer;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Shooter.ShooterState;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.base.TorqueDirection;

public final class Shoot extends TorqueCommand implements Subsystems {
    private final double rpm, hood, targetYaw, time;
    private double start = -1;
    private final boolean stop;

    public Shoot(final double rpm, final double hood, final boolean stop, final double time) {
        this(rpm, hood, Double.NaN, stop, time);
    }

    public Shoot(final double rpm, final double hood, final double targetYaw, final boolean stop, final double time) {
        this.rpm = rpm;
        this.hood = hood;
        this.targetYaw = targetYaw;
        this.time = time;
        this.stop = stop;
    }

    @Override
    protected final void init() {
        shooter.setState(ShooterState.SETPOINT);
        shooter.setFlywheelSpeed(rpm);
        shooter.setHoodPosition(hood);
        drivebase.setTargetYaw(targetYaw);
    }

    @Override
    protected final void continuous() {
        if (shooter.isReady() && start == -1)
            start = Timer.getFPGATimestamp();
    }

    @Override
    protected final boolean endCondition() {
        return start != -1 && (Timer.getFPGATimestamp() - start) > time;
    }

    @Override
    protected final void end() {
        if (stop)
            shooter.setState(ShooterState.OFF);
        else {
            shooter.setState(ShooterState.SETPOINT);
            shooter.setFlywheelSpeed(1000);
        }

        magazine.setGateDirection(TorqueDirection.OFF);
    }
}   
