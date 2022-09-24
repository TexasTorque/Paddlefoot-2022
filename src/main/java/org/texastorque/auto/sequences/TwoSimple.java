/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Input;
import org.texastorque.Subsystems;
import org.texastorque.auto.commands.DriveForTime;
import org.texastorque.auto.commands.Shoot;
import org.texastorque.subsystems.Intake.IntakeState;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TwoSimple extends TorqueSequence implements Subsystems {
    private double rpm1 = 1775, hood1 = 25;

    public TwoSimple() {
        addBlock(new TorqueBlock(new TorqueExecute(() -> {
            drivebase.getOdometry().resetPosition(new Pose2d(6.51, 5.3, Rotation2d.fromDegrees(158)),
                    Rotation2d.fromDegrees(158));
        })));

        addBlock(new TorqueBlock(new TorqueExecute(() -> {
            intake.setState(IntakeState.INTAKE);
        })));
        // Drive back and pick up the second ball
        addBlock(new TorqueBlock(new DriveForTime(1., -8)));
        // Align and shoot the balls 
        addBlock(new TorqueBlock(new Shoot(rpm1, hood1, 5.2)));

        addBlock(new TorqueBlock(new TorqueExecute(() -> {
            Input.getInstance().invertDrivebaseControls();
        })));

    }
}
