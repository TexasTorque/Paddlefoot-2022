/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Clutch-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.Drive;
import org.texastorque.auto.commands.Path;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TwoSimple extends TorqueSequence implements Subsystems {

    public TwoSimple() {
        addBlock(new TorqueBlock(new TorqueExecute(() -> {
                drivebase.getOdometry().resetPosition(new Pose2d(6.1, 5.2, Rotation2d.fromDegrees(135)), Rotation2d.fromDegrees(135));
        })));
        addBlock(new TorqueBlock(new Drive(-50., -8)));
        // addBlock(new TorqueBlock(new Path("Jacob", true, 4, 4)));
    }
}
