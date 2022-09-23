/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;

public final class DriveForTime extends TorqueCommand implements Subsystems {
    private double time, startTime;
    private ChassisSpeeds speeds;
    private final Pose2d initial;

    public DriveForTime(final double time, final double xSpeed) {
        this.time = time;
        this.speeds = new ChassisSpeeds(xSpeed, 0, 0);
        this.initial = drivebase.getPose();
    }

    @Override
    protected final void init() {
        drivebase.setSpeeds(speeds);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected final void continuous() {
        final double deltaX = drivebase.getPose().getX() - initial.getX();
        final double deltaY = drivebase.getPose().getY() - initial.getY();
        final double displacment = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        SmartDashboard.putNumber("Drive ∆X", deltaX);
        SmartDashboard.putNumber("Drive ∆Y", deltaY);
        SmartDashboard.putNumber("Drive ∆H", displacment);
    }

    @Override
    protected final boolean endCondition() {
        return time != -1 && (Timer.getFPGATimestamp() - startTime >= time);

    }

    @Override
    protected final void end() {
        drivebase.setSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}