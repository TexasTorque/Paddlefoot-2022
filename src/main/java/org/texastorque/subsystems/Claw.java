/* Copyright 2022 Texas Torque.
**
* This file is part of Paddlefoot-2022, which is not licensed for distribution.
* For more details, see ./license.txt or write <jus@gtsbr.org>.
*/

package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueTimeout;
import org.texastorque.torquelib.motors.TorqueSparkMax;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Claw extends TorqueSubsystem implements Subsystems {
    private static volatile Claw instance;

    public static enum ClawMode {
        BALL(true, 1, 12.), HATCH(false, -1, 12.);

        public final boolean isJawOpen;
        public final int direction;
        public final double currentLevel;

        private ClawMode(final boolean isJawOpen, final int direction, final double currentLevel) {
            this.isJawOpen = isJawOpen;
            this.direction = direction;
            this.currentLevel = currentLevel;
        }
    };

    private ClawMode mode = ClawMode.HATCH;

    public final void setMode(final ClawMode mode) {
        this.mode = mode;
    }

    public final void setModeSafe(final ClawMode mode) {
        if (mode != this.mode && !hasItem)
            this.mode = mode;
    }

    public final ClawMode getMode() { return mode; }

    public static enum ClawState {
        INTAKE(1), OUTTAKE(-1), OFF(0);

        public final double speed;

        private ClawState(final double speed) { this.speed = speed; }
    };

    private ClawState state = ClawState.OFF;

    public final void setState(final ClawState state) { this.state = state; }

    private final TorqueSparkMax roller;
    private final Solenoid jaw;

    private Claw() {
        roller = new TorqueSparkMax(Ports.CLIMBER.HATCH);
        jaw = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }


    private int powerSpikes = 0;
    private final TorqueClick spikeClick = new TorqueClick();
    private boolean hasItem = false;

    private TorqueTimeout rumbleTimeout = new TorqueTimeout(1);

    @Override
    public final void update(final TorqueMode _mode) {
        SmartDashboard.putString("Claw State", state.toString());
        SmartDashboard.putString("Claw Mode", mode.toString());
        SmartDashboard.putNumber("Current", roller.getCurrent());
        SmartDashboard.putNumber("Spike", powerSpikes);

        if (state == ClawState.OFF) {
            if (spikeClick.calculate(roller.getCurrent() >= mode.currentLevel))
                powerSpikes++;
        } else 
            powerSpikes = 0;

        if (state == ClawState.OUTTAKE)
            hasItem = false;

        if (powerSpikes >= 2 || hasItem) {
            hasItem = true;
            roller.setPercent(0);
        }  else 
            roller.setPercent(state.speed * mode.direction);

        jaw.set(mode.isJawOpen);

        final boolean rumble = rumbleTimeout.calculate(hasItem);

        Input.getInstance().getDriver().setRumble(rumble);
        Input.getInstance().getOperator().setRumble(rumble);

        state = ClawState.OFF;
    }

    public static final synchronized Claw getInstance() {
        return instance == null ? instance = new Claw() : instance;
    }
}
