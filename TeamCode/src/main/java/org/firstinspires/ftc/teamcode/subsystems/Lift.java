package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.Utils;

@Config
public class Lift {

    private DcMotor lLift, rLift;
    private RevTouchSensor reset;

    private double target = 0, power, ticks, lastPower;

    public static double p = 0.01, i, d;

    public PIDController controller = new PIDController(p, i, d);

    private boolean manual = false;

    public enum Mode {
        target,
        manual,
        hang
    }

    Mode runMode = Mode.target, lastMode = runMode;

    public Lift(HardwareMap hardwareMap, boolean manual) {
        rLift = hardwareMap.dcMotor.get("rLift");
        lLift = hardwareMap.dcMotor.get("lLift");
        lLift.setDirection(DcMotorSimple.Direction.REVERSE);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        reset = hardwareMap.get(RevTouchSensor.class, "lReset");

        controller.setTolerance(10);

        if (manual) {
            runMode = Mode.manual;
        } else {
            runMode = Mode.target;
        }
    }

    public double readTicks(double ticks) {
        if (reset.isPressed()) {
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return 0;
        } else {
            return ticks;
        }
    }

    public void setTarget(double target) {
        this.target = target;
    }

    private double mInput = 0;

    public void setManualInput(double input) {
        mInput = input;
    }

    private double l1 = 2600, l2 = 1600, limit = l2;

    public void setLimit(boolean vertical) {
        if (vertical) {
            limit = l1;
        } else {
            limit = l2;
        }
    }

    public static double hangPower = .45, pullPower = 0.6;

    public void setRunMode(Mode runMode) {
        this.runMode = runMode;
    }

    public void update() {
        ticks = readTicks(Math.abs(lLift.getCurrentPosition()));

        if (runMode == Mode.manual) {
            double dt = 45*mInput;
            if (target + dt > limit) {
                target = limit;
            } else if (target + dt < 0) {
                target = 0;
            } else {
                target += dt;
            }
        } else if (runMode == Mode.hang) {
            apply(hangPower);
            return;
        }

        if (target >= limit) {
            target = limit;
        } else if (target < 0) {
            target = 0;
        }

        controller.setSetPoint(target);

        power = controller.calculate(ticks);

        if (target == 0 && reset.isPressed()) {
            apply(0);
        } else {
            apply(power * override);
        }

        lastMode = runMode;
    }

    public double getTicks() {
        return ticks;
    }

    private void apply(double p) {
        lLift.setPower(p);
        rLift.setPower(p);
    }

    public boolean check() {
        return controller.atSetPoint();
    }

    double override = 1;

    public void enableOverride(boolean enable) {
        if (enable) {
            override = 0;
        } else {
            override = 1;
        }
    }

    public boolean readSensor() {
        return reset.isPressed();
    }

}
