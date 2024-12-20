package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class ArmCommand extends CommandBase {

    private Deposit deposit;

    public static double floorSpecArm = 0.63, floorSpecPitch = 0.3;
    public enum DepositState {
        floorIntake,
        specIntake,
        basketDepo,
        specDepo,
        specSlam,
        floorSpec
    }

    public enum ClawState {
        open,
        closed
    }

    public enum WristState {
        horizontal,
        vertical,
        wrapped
    }

    private ElapsedTime runtime = new ElapsedTime();
    private double wrist, arm, pitch, claw;

    boolean f = true;

    private DepositState depositState; private ClawState clawState; private WristState wristState;

    public ArmCommand(Deposit deposit, DepositState depositState, ClawState clawState, WristState wristState) {
        this.clawState = clawState; this.depositState = depositState; this.deposit = deposit; this.wristState = wristState;
    }

    @Override
    public void initialize() {

        switch (depositState) {
            case floorIntake:
                arm = Deposit.armBack;
                pitch = Deposit.aPitchBack;
                break;
            case specIntake:
                arm = Deposit.armSpec;
                pitch = Deposit.aPitchSpec;
                break;
            case specDepo:
                arm = Deposit.armDown;
                pitch = Deposit.aPitchDown;
                break;
            case basketDepo:
                arm = Deposit.armUp;
                pitch = Deposit.aPitchUp;
                break;
            case specSlam:
                arm = Deposit.specSlam;
                pitch = Deposit.specSlamPitch;
                break;
            case floorSpec:
                arm = floorSpecArm;
                pitch = floorSpecPitch;
                break;
        }

        switch (clawState) {
            case open:
                claw = Claw.open;
                break;
            case closed:
                claw = Claw.closed;
                break;
        }

        switch (wristState) {
            case wrapped:
                wrist = 0.94;
                break;
            case vertical:
                wrist = 0.5;
                break;
            case horizontal:
                wrist = 0;
                break;
        }
    }

    @Override
    public void execute() {
        deposit.setDeposit(wrist, pitch, arm, claw);
    }

    @Override
    public boolean isFinished() {
        if (f) {
            runtime.reset();
            f = false;
        }
        return runtime.seconds() > 0.3;
    }

}
