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

        this.manual = manual;
    }

    public double readTicks(double ticks) {
        if (reset.isPressed()) {
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    private double l1 = 2592, l2 = 1600, limit = l2;

    public void setLimit(boolean vertical) {
        if (vertical) {
            limit = l1;
        } else {
            limit = l2;
        }
    }



    public void update() {
        ticks = readTicks(Math.abs(lLift.getCurrentPosition()));
        if (manual) {
            double dt = 45*mInput;
            target += dt;
        }

        if (target >= limit) {
            target = limit;
        } else if (target <= 0) {
            target = 0;
        }

        controller.setSetPoint(target);

        power = controller.calculate(ticks);

        if (Utils.valInThresh(power, lastPower, 0.001)) {
            if (target == 0 && reset.isPressed()) {
                power = 0;
            }
            apply(power);
            lastPower = power;
        }
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

}
