/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Clutch-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;

public final class Drive extends TorqueCommand implements Subsystems {
    private final double distance;
    private ChassisSpeeds speeds;
    private final Pose2d initial;

    public Drive(final double distance, final double x) {
        this.distance = distance;
        this.speeds = new ChassisSpeeds(x, 0, 0);
        this.initial = drivebase.getPose();
    }

    @Override
    protected final void init() {
        drivebase.setSpeeds(speeds);
    }

    @Override
    protected final void continuous() {
        // SmartDashboard.putNumber("Disp_", drivebase.getDisplacement());
        final double deltaX = drivebase.getPose().getX() - initial.getX();
        final double deltaY = drivebase.getPose().getY() - initial.getY();
        final double displacment = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        SmartDashboard.putNumber("Drive ∆X", deltaX);
        SmartDashboard.putNumber("Drive ∆Y", deltaY);
        SmartDashboard.putNumber("Drive ∆H", displacment);
        SmartDashboard.putBoolean("Drive isgud", Math.abs(drivebase.getDisplacement()) >= Math.abs(distance));
    }

    @Override
    protected final boolean endCondition() {
        final double deltaX = drivebase.getPose().getX() - initial.getX();
        final double deltaY = drivebase.getPose().getY() - initial.getY();
        final double displacment = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // return Math.abs(displacment) >= Math.abs(distance);
        return Math.abs(drivebase.getDisplacement()) >= Math.abs(distance);

    }

    @Override
    protected final void end() {
        drivebase.setSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}