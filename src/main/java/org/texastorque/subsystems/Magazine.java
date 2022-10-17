/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Paddlefoot-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.base.TorqueSubsystemState;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.util.TorqueMath;

public final class Magazine extends TorqueSubsystem implements Subsystems {
    private static volatile Magazine instance;

    public static enum MagazineState {
        OFF,
        OUT,
        IDLE,
        POP_MINS;
    }
    public static final double FLYWHEEEL_MAX = 3000;
    private final TorqueSparkMax belt, gate, hood, flywheelLeft, flywheelRight;
    public TorqueDirection beltDirection, gateDirection;
    public MagazineState state;
    
    private Magazine() {
        belt = new TorqueSparkMax(Ports.MAGAZINE.BELT);
        belt.configureDumbCANFrame();
        gate = new TorqueSparkMax(Ports.MAGAZINE.GATE);
        gate.configureDumbCANFrame();

        state = MagazineState.OFF;
        beltDirection = TorqueDirection.OFF;
        gateDirection = TorqueDirection.OFF;

        // Stuff from shooter
        flywheelLeft = new TorqueSparkMax(Ports.SHOOTER.FLYWHEEL.LEFT);
        flywheelRight = new TorqueSparkMax(Ports.SHOOTER.FLYWHEEL.RIGHT);
        flywheelLeft.configurePID(TorquePID.create(.000005).addFeedForward(.00034).setTolerance(20).build());
        hood = new TorqueSparkMax(Ports.SHOOTER.HOOD);
                hood.configurePID(TorquePID.create(.1)
                .addIntegral(.001)
                .addOutputRange(-.7, .7)
                .addIntegralZone(.3)
                .build());

        hood.configurePositionalCANFrame();
        hood.burnFlash();
    }

    public final void setState(final MagazineState state) {
        this.state = state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }


    @Override
    public final void update(final TorqueMode mode) {
        if (intake.isOutaking()) {
            state = MagazineState.OUT;
        }
        else if (state == MagazineState.POP_MINS) {
           beltDirection = TorqueDirection.FORWARD;
           gateDirection = TorqueDirection.FORWARD;
        }
        else if (state == MagazineState.OUT) {
           beltDirection = TorqueDirection.REVERSE;
           gateDirection = TorqueDirection.REVERSE;
        } 
        else {
            beltDirection = TorqueDirection.FORWARD;
            gateDirection = TorqueDirection.OFF;
        }

        // Idle shooter flywheels for MM
        if (state != MagazineState.OFF) {
            flywheelLeft.setVelocityRPM(clampRPM(200));
            flywheelRight.setVoltage(-flywheelLeft.getVoltage());
        } else {
            flywheelLeft.setVoltage(0);
            flywheelRight.setVoltage(0);
        }

        

        belt.setPercent(beltDirection.get());
        gate.setPercent(gateDirection.get());
      
        SmartDashboard.putNumber("Gate", gateDirection.get());
        SmartDashboard.putNumber("Belt Amps", belt.getCurrent());
      
        TorqueSubsystemState.logState(beltDirection);
        TorqueSubsystemState.logState(gateDirection);

        
    }
    private final double clampRPM(final double rpm) {
        return TorqueMath.constrain(rpm, 0, FLYWHEEEL_MAX);
    }
    public static final synchronized Magazine getInstance() {
        return instance == null ? instance = new Magazine() : instance;
    }
}
