/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Clutch-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.util.TorqueMath;

public final class Drive extends TorqueCommand implements Subsystems {
    private final double distance;
    private ChassisSpeeds speeds;

    public Drive(final double distance, final double x) {
        this.distance = distance;
        this.speeds = new ChassisSpeeds(x, 0, 0);
    }

    @Override
    protected final void init() {
        drivebase.setSpeeds(speeds);
    }

    @Override
    protected final void continuous() {
        SmartDashboard.putNumber("Disp_", drivebase.getDisplacement());
    }

    @Override
    protected final boolean endCondition() {
        // return drivebase.getDisplacement() >= distance;
        return TorqueMath.toleranced(drivebase.getDisplacement(), distance, 10);
    }

    @Override
    protected final void end() {
        drivebase.setSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}