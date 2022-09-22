/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.Drive;
import org.texastorque.auto.commands.Shoot;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public final class TwoSimple extends TorqueSequence implements Subsystems {
    private double rpm1 = 1200, hood1 = 30, targetYaw = 3;



    public TwoSimple() {
        addBlock(new TorqueBlock(new TorqueExecute(() -> {
                drivebase.getOdometry().resetPosition(new Pose2d(6.1, 5.2, Rotation2d.fromDegrees(135)), Rotation2d.fromDegrees(135));
        })));

        // Drive back and pick up the second ball
        addBlock(new TorqueBlock(new Drive(-50., -8)));
        // Align and shoot the balls 
        addBlock(new TorqueBlock(new Shoot(rpm1, hood1, targetYaw, false, 1.4)));
    }
}
