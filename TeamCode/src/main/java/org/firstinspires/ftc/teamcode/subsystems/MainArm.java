package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Deprecated
public class MainArm {

    private MPPivot pivot; private Lift lift;

    public enum State {
        intake,
        basket,
        backpickup,
        inter,
        hang,
        silly
    }

    private boolean override = false;

    boolean flag = false;

    public static double vertAngle = 90, backUpAngle = 90, specimen = 19, hangle = 45, silly = 90;

    public MainArm(HardwareMap hardwareMap, boolean manual) {
        pivot = new MPPivot(hardwareMap); lift = new Lift(hardwareMap, manual);
        flag = manual;
        pivot.setTargetAngle(0); lift.setTarget(0); lift.setLimit(false);
    }

    private State mainState = State.intake, lState = mainState;

    public void setMainState(State mainState) {
        this.mainState = mainState;
    }

    private double lmi = 0;

    public void setLmi(double lmi) {
        this.lmi = lmi;
    }

    private double ltg = 0, lltg = ltg;

    public void setLtg(double ltg) {
        this.ltg = ltg;
    }

    public void update() {
        if (mainState != lState) {
            switch (mainState) {
                case basket:
                    pivot.setTargetAngle(vertAngle);
                    lift.setLimit(true);
                    break;
                case intake:
                    pivot.setTargetAngle(0);
                    lift.setLimit(false);
                    break;
                case inter:
                    pivot.setTargetAngle(specimen);
                    lift.setLimit(true);
                    break;
                case backpickup:
                    pivot.setTargetAngle(backUpAngle);
                    lift.setLimit(true);
                    break;
                case hang:
                    pivot.setTargetAngle(hangle);
                    lift.setLimit(true);
                    break;
                case silly:
                    pivot.setTargetAngle(silly);
                    lift.setLimit(true);
            }
        }

        lState = mainState;

        if (mode != lastMode) {
            lift.setRunMode(mode);
        }

        if (flag && !override) {
            if (lltg != ltg) {
                lift.setTarget(ltg);
            } else {
                lift.setManualInput(lmi);
            }
        } else {
            lift.setManualInput(0);
            lift.setTarget(ltg);
        }


        pivot.update();
        lift.update();
        lastMode = mode;
        lltg = ltg;
    }

    public boolean stateComplete() {
        return lift.check() && pivot.check();
    }

    public boolean lCheck() {
        return lift.check();
    }

    Lift.Mode mode = Lift.Mode.target, lastMode = Lift.Mode.target;

    public void setLiftMode(Lift.Mode mode) {
        this.mode = mode;
    }

    public void setOverride(boolean override) {
        this.override = override;
    }
    public boolean readLiftReset() {
        return lift.readSensor();
    }

    public double liftTicks() {
        return lift.ticks;
    }

}
