package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class ConfigArmCommand extends CommandBase {

    private Deposit deposit;

    private double flagTime = 0.3;

    public static double floorSpecArm = 0.63, floorSpecPitch = 0.3;
    public enum DepositState {
        floorIntake,
        specIntake,
        basketDepo,
        specDepo,
        specSlam,
        floorSpec,
        shoot,
        antiSlamDunkTechnology
    }

    public enum ClawState {
        open,
        closed
    }

    public enum WristState {
        horizontal,
        vertical,
        wrapped,
        diagonal,
        diagonal2,
        current
    }

    private ElapsedTime runtime = new ElapsedTime();
    private double wrist, arm, pitch, claw;

    boolean f = true;

    private DepositState depositState; private ClawState clawState; private WristState wristState;

    public ConfigArmCommand(Deposit deposit, DepositState depositState, ClawState clawState, WristState wristState, double flagTime) {
        this.clawState = clawState; this.depositState = depositState; this.deposit = deposit; this.wristState = wristState;
        this.flagTime = flagTime;
    }

    @Override
    public void initialize() {

        switch (depositState) {
            case floorIntake:
                arm = Deposit.armBack;
                pitch = Deposit.aPitchBack;
                break;
            case specIntake:
                arm = 0.45;
                pitch = 0.34;
                break;
            case specDepo:
                arm = Deposit.armDown;
                pitch = Deposit.aPitchDown;
                break;
            case basketDepo:
                arm = Deposit.armUp;
                pitch = Deposit.aPitchUp;
                break;
            case antiSlamDunkTechnology:
                arm = Deposit.armUp;
                pitch = 0.55;
                break;
            case specSlam:
                arm = 0;
                pitch = 0.2;
                break;
            case floorSpec:
                arm = floorSpecArm;
                pitch = floorSpecPitch;
                break;
            case shoot:
                arm = 0.48;
                pitch = 0.58;
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
                wrist = 0.055;
                break;
            case diagonal:
                wrist = 0.25;
                break;
            case diagonal2:
                wrist = 0.9;
                break;
            case current:
                break;
        }
    }

    @Override
    public void execute() {
        if (wristState != WristState.current) {
            deposit.setDeposit(wrist, pitch, arm, claw);
        } else {
            deposit.setDeposit(deposit.wristP(), pitch, arm, claw);
        }
    }

    @Override
    public boolean isFinished() {
        if (f) {
            runtime.reset();
            f = false;
        }
        return runtime.seconds() > flagTime;
    }

}
