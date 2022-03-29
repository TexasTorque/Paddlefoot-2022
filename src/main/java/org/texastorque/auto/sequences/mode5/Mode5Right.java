package org.texastorque.auto.sequences.mode5;

import com.pathplanner.lib.PathPlanner;

import org.texastorque.auto.commands.*;
import org.texastorque.subsystems.Intake.IntakeDirection;
import org.texastorque.subsystems.Intake.IntakePosition;
import org.texastorque.subsystems.Magazine.BeltDirections;
import org.texastorque.subsystems.Magazine.GateSpeeds;
import org.texastorque.torquelib.auto.*;

public class Mode5Right extends TorqueSequence {
        public Mode5Right(String name) {
                super(name);

                init();
        }

        @Override
        protected void init() {
                // Start Intake, Automag
                addBlock(new TorqueBlock(
                                new SetIntake(IntakePosition.DOWN, IntakeDirection.INTAKE),
                                new SetMagazine(BeltDirections.INTAKE, GateSpeeds.CLOSED), 
                                new PrepareTurret(-75),
                                new PrepareShooter(50, 1800),
                                new Pathplanner("Mode5Right_1")));

                // Shoot preload and pickup
                addBlock(new TorqueBlock(new ShootConst(1800, 50, -75, false, 1)));

                // Go to human player
                addBlock(new TorqueBlock(new Pathplanner("Mode5Right_2", false)));

                // Wait for human player
                addBlock(new TorqueBlock(new Wait(1)));

                // Go to the last shoot
                addBlock(new TorqueBlock(new PrepareTurret(-5), new PrepareShooter(50, 1975),
                                new Pathplanner("Mode5Right_3", false)));

                // Shoot!
                addBlock(new TorqueBlock(new ShootConst(1975, 50, -5, false, 1.6)));


                // Shut off
                addBlock(new TorqueBlock(
                                new SetIntake(IntakePosition.PRIME, IntakeDirection.STOPPED),
                                new SetMagazine(BeltDirections.OFF, GateSpeeds.CLOSED)));
        }
}
