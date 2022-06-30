package org.texastorque.auto;

import org.texastorque.auto.sequences.Five;
import org.texastorque.auto.sequences.Omar;
import org.texastorque.auto.sequences.One;
import org.texastorque.auto.sequences.OneEvil;
import org.texastorque.auto.sequences.Two;
import org.texastorque.auto.sequences.TwoEvil;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        addSequence("Omar", new Omar());
        addSequence("One", new One());
        addSequence("One (evil)", new OneEvil());
        addSequence("Two", new Two());
        addSequence("Two (evil)", new TwoEvil());
        addSequence("Five", new Five());
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}