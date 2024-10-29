package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MainArm {

    private MPPivot pivot; private Lift lift;

    public enum State {
        intake,
        basket,
        backpickup,
        inter
    }

    public static double hLimit = 0, vlimit = 0;

    public static double vertAngle = 117, backUpAngle = 130, specimen = 30;

    private boolean flag = false;

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

    private double ltg = 0;

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
            }
        }

        lState = mainState;

        if (flag) {
            lift.setManualInput(lmi);
        } else {
            lift.setTarget(ltg);
        }

        pivot.update();
        lift.update();
    }

    public State getlState() {
        return lState;
    }

    public double getAngle() {
        return pivot.angle;
    }

    public boolean stateComplete() {
        return lift.check() && pivot.check();
    }
}
