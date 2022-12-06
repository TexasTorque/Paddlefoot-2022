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
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.motors.legacy.TorqueSparkMax;

import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Elevator extends TorqueSubsystem implements Subsystems {
    private static volatile Elevator instance;

    public static final double LIFT_UP = 110., LIFT_BOTTOM = 0, VOLTS = 5;

    private final TorqueNEO lift;

    private ElevatorState state = ElevatorState.OFF;

    public enum ElevatorState {
        POSITION, EXTEND, RETRACT, OFF;
    }

    public void setState(final ElevatorState state) {
        this.state = state;
    }

    private TorqueDirection liftDirection = TorqueDirection.OFF;

    private final TorquePID pid = TorquePID.create(.01).build();

    private Elevator() {
        lift = new TorqueNEO(Ports.CLIMBER.LIFT.RIGHT);
        lift.addFollower(Ports.CLIMBER.LIFT.LEFT, true);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    public void setLiftPos(double position) {
    }

    @Override
    public final void update(final TorqueMode mode) {

        SmartDashboard.putNumber("LIFTDIR", liftDirection.get());
        SmartDashboard.putNumber("Climber Lift Position", lift.getPosition());

        if (state == ElevatorState.EXTEND)
            if (lift.getPosition() > LIFT_UP)
                stop();
            else
                lift.setVolts(VOLTS);
        else if (state == ElevatorState.RETRACT)
            if (lift.getPosition() < LIFT_BOTTOM)
                stop();
            else
                lift.setVolts(-VOLTS);
        else if (state == ElevatorState.OFF) stop();

    }

    private void stop() {
        lift.setVolts(0);
    }

    public static final synchronized Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
