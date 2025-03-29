package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift {

    private DcMotorEx lLift, rLift;
    private RevTouchSensor reset;

    public double target = 0, power, ticks, lastPower, lastTarget = target, velocity = 0, indexedPosition = 0;

    public static double p = 0.01, i, d=0, f = 0;

    public static double hp = 0.1, hi, hd, hf;
    public PIDController controller = new PIDController(p, i, d);

    public static double vel = 8000, accel = 9000;
    private TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(vel, accel);
    private TrapezoidProfile profile;

    public enum Mode {
        target,
        manual,
        hang
    }

    private Mode runMode = Mode.target, lastMode = runMode;

    public Lift(HardwareMap hardwareMap, boolean manual) {
        rLift = ((DcMotorEx) hardwareMap.dcMotor.get("rLift"));
        lLift = ((DcMotorEx) hardwareMap.dcMotor.get("lLift"));
        rLift.setDirection(DcMotorSimple.Direction.REVERSE);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        reset = hardwareMap.get(RevTouchSensor.class, "lReset");

        controller.setTolerance(35);

        if (manual) {
            runMode = Mode.manual;
        } else {
            runMode = Mode.target;
        }
        profile = new TrapezoidProfile(liftConstraints, new TrapezoidProfile.State(0,0)); compEnabled = false;
    }

    private VoltageSensor voltageSensor; private boolean compEnabled = false;

    public Lift(HardwareMap hardwareMap, boolean manual, VoltageSensor voltageSensor) {
        rLift = ((DcMotorEx) hardwareMap.dcMotor.get("rLift"));
        lLift = ((DcMotorEx) hardwareMap.dcMotor.get("lLift"));
        rLift.setDirection(DcMotorSimple.Direction.REVERSE);

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

        this.voltageSensor = voltageSensor; compEnabled = true;

        profile = new TrapezoidProfile(liftConstraints, new TrapezoidProfile.State(0,0));
    }

    public void setPIDTolerance(double tol) {
        controller.setTolerance(tol);
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

    public static double l1 = 867, l2 = 480, limit = l2;

    public void setLimit(boolean vertical) {
        if (vertical) {
            limit = l1;
        } else {
            limit = l2;
        }
    }

    public void setRunMode(Mode runMode) {
        this.runMode = runMode;
    }

    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    public void setPID(double p) {
        controller.setPID(p, 0, d);
    }

    public void update() {
        ticks = readTicks(Math.abs(lLift.getCurrentPosition()));

        if (runMode == Mode.manual) {
            target = target + (50 * mInput);
            if (target >= limit) {
                target = limit;
            } else if (target < 0) {
                target = 0;
            }
            controller.setSetPoint(target);
        } else if (runMode == Mode.hang) {
        } else {
            if (target != lastTarget) {
                if (target >= limit) {
                    target = limit;
                } else if (target < 0) {
                    target = 0;
                }
                liftConstraints = new TrapezoidProfile.Constraints(vel, accel);
                profile = new TrapezoidProfile(liftConstraints, new TrapezoidProfile.State(target, 0),
                        new TrapezoidProfile.State(ticks, velocity));
                fullTimer.reset();
            }

            indexedPosition = profile.calculate(fullTimer.seconds()).position;
            controller.setSetPoint(indexedPosition);
        }

        power = controller.calculate(ticks);

        if (target == 0 && reset.isPressed()) {
            power = 0;
            apply(power);
        } else {
            apply(power);
            lastPower = power;
        }

        lastTarget = target;
        velocity = (target-lastTarget)/velTimer.seconds();
        velTimer.reset();
    }

    public double getTicks() {
        return ticks;
    }

    public void apply(double p) {
        if (compEnabled) {
            double comp = Robot.universalVoltage/voltageSensor.getVoltage();
            rLift.setPower(p * comp);
            lLift.setPower(p * comp);
        } else {
            rLift.setPower(p);
            lLift.setPower(p);
        }
    }

    public boolean check() {
        return controller.atSetPoint();
    }

    public boolean readSensor() {
        return reset.isPressed();
    }

    public double readCurrent() {
        return (lLift.getCurrent(CurrentUnit.AMPS) + rLift.getCurrent(CurrentUnit.AMPS))/2;
    }

}
