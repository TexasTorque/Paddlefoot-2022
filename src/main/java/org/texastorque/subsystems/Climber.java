/**
 * Copyright 2022 Texas Torque.
 * 
 * This file is part of Clutch-2022, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@gtsbr.org>.
 */
package org.texastorque.subsystems;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;

public final class Climber extends TorqueSubsystem implements Subsystems {
    private static volatile Climber instance;

    private Climber() {
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    @Override
    public final void update(final TorqueMode mode) {
    }

    public final boolean hasStarted() { return false; }

    public static final synchronized Climber getInstance() {
        return instance == null ? instance = new Climber() : instance;
    }
}
