package org.texastorque.auto.sequences.assists;

import org.texastorque.auto.commands.*;
import org.texastorque.torquelib.auto.*;

public class RotateToBall extends TorqueSequence {

    public RotateToBall() {
        super("RotateToBall");
        init();
    }

    @Override
    protected void init() {
        addBlock(new TorqueBlock(new BallRotator()));
    }
}
