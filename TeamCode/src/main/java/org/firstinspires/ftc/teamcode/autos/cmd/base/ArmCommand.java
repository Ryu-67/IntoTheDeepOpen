package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

@Config
public class ArmCommand extends CommandBase {

    private Deposit deposit;

    public static double floorSpecArm = 0.63, floorSpecPitch = 0.3;
    public enum DepositState {
        floorIntake,
        specIntake,
        basketDepo,
        specDepo,
        specSlam,
        floorSpec,
        shoot,
        dropSlam,
        wallSpecUp,
        wallSpecTele,
        hell,
        antiSlamDunkTechnology,
        preIntakeLifted,
        extralowground
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
            case wallSpecTele:
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
            case antiSlamDunkTechnology:
                arm = Deposit.armUp;
                pitch = 0.55;
                break;
            case specSlam:
                arm = Deposit.specSlam;
                pitch = Deposit.specSlamPitch;
                break;
            case floorSpec:
                arm = floorSpecArm;
                pitch = floorSpecPitch;
                break;
            case shoot:
                arm = 0.48;
                pitch = 0.58;
                break;
            case dropSlam:
                arm = Deposit.slamPos;
                pitch = Deposit.slamOnPitch;
                break;
            case wallSpecUp:
                arm = 0.805;
                pitch = 0.625;
                break;
            case preIntakeLifted:
                arm = 0.24;
                pitch = 0.075;
                break;
            case extralowground:
                arm = 0.22;
                pitch = Deposit.aPitchBack;
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
            deposit.setDeposit(wrist, pitch, arm, deposit.wristP());
        }
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
