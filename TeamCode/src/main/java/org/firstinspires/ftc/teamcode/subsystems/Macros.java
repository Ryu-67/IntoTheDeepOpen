package org.firstinspires.ftc.teamcode.subsystems;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

public class Macros {

    public static StateMachine backIn(MainArm arm, Deposit deposit, MainArm.State stateInput) {
        return new StateMachineBuilder()
                .state("up")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .transitionTimed(0.4)
                .state("retract")
                .onEnter(()-> {arm.setOverride(true); arm.setLtg(0);})
                .transition(arm::lCheck)
                .onExit(()->{arm.setLiftOverride(true); arm.setOverride(false);})
                .state("pivot")
                .onEnter(()->arm.setMainState(stateInput))
                .transition(arm::stateComplete)
                .state("neutral")
                .onEnter(()->arm.setLiftOverride(false))
                .build();
    }

    public static StateMachine backInA(MainArm arm, Deposit deposit, MainArm.State stateInput) {
        return new StateMachineBuilder()
                .state("up")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open))
                .transitionTimed(0.2)
                .state("retract")
                .onEnter(()-> {arm.setLtg(0);})
                .transition(arm::lCheck)
                .onExit(()->{arm.setLiftOverride(true);})
                .state("pivot")
                .onEnter(()->arm.setMainState(stateInput))
                .transition(arm::stateComplete)
                .state("neutral")
                .onEnter(()->arm.setLiftOverride(false))
                .build();
    }

}
