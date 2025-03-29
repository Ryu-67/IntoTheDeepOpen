package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class MPPivot {

    private DcMotor lPivot, rPivot;

    private RevTouchSensor reset;

    private PIDController controller = new PIDController(p, i, d);
    private TrapezoidProfile profile;
    TrapezoidProfile.Constraints upConstraints = new TrapezoidProfile.Constraints(3*Math.PI, 20*Math.PI), downConstraints = new TrapezoidProfile.Constraints(5*Math.PI, 3*Math.PI);
    public static double p = 3.5, i = 0, d = 0.05, k = 0.1;

    public double tpr = (537.7*(2.66666666666666667)*(2))/(2*Math.PI);
    public boolean flag = false;

    public double updateCurrentAngle(double current) {
        if (reset.isPressed()) {
            rPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flag = true;
            return  0;
        } else {
            flag = false;
            return  current/tpr;
        }
    }

    public double angle = 0, langle = angle, aVelocity = 0;

    public MPPivot(HardwareMap hardwareMap) {
        rPivot = hardwareMap.dcMotor.get("rPivot");
        lPivot = hardwareMap.dcMotor.get("lPivot");

        rPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        lPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset = hardwareMap.get(RevTouchSensor.class, "reset");

        profile = new TrapezoidProfile(upConstraints, new TrapezoidProfile.State(0, 0));
    }

    private VoltageSensor voltageSensor; private boolean compEnabled = false;

    public MPPivot(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        rPivot = hardwareMap.dcMotor.get("rPivot");
        lPivot = hardwareMap.dcMotor.get("lPivot");

        rPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        lPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset = hardwareMap.get(RevTouchSensor.class, "reset");

        profile = new TrapezoidProfile(upConstraints, new TrapezoidProfile.State(0, 0));

        this.voltageSensor = voltageSensor; compEnabled = true;
    }

    public double targetAngle = 0, power = 0, lastPower = power, lta = targetAngle, indexedPosition = 0;

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = Math.toRadians(targetAngle);
    }

    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    public void update() {

        angle = Math.abs(updateCurrentAngle(rPivot.getCurrentPosition()));

        if (targetAngle != lta && targetAngle > lta) {
            profile = new TrapezoidProfile(upConstraints, new TrapezoidProfile.State(targetAngle, 0), new TrapezoidProfile.State(angle, aVelocity));
            fullTimer.reset();
        } else if (targetAngle != lta && targetAngle < lta) {
            profile = new TrapezoidProfile(downConstraints, new TrapezoidProfile.State(targetAngle, 0), new TrapezoidProfile.State(angle, aVelocity));
            fullTimer.reset();
        }

        indexedPosition = profile.calculate(fullTimer.seconds()).position;

        controller.setSetPoint(indexedPosition);

        power = controller.calculate(angle) + (k * Math.cos(targetAngle));

        if (flag && targetAngle == 0) {
            apply(0);
        } else  {
            apply(power);
            lastPower = power;
        }

        lta = targetAngle;

        aVelocity = (angle-langle)/velTimer.seconds();
        langle = angle;
        velTimer.reset();
    }

    public double readRawMotorTicks() {
        return rPivot.getCurrentPosition();
    }

    public void apply(double p) {
        if (compEnabled) {
            double comp = Robot.universalVoltage/voltageSensor.getVoltage();
            rPivot.setPower(p * comp);
            lPivot.setPower(p * comp);
        } else {
            rPivot.setPower(p);
            lPivot.setPower(p);
        }
    }

    public boolean check() {
        return Math.abs(Math.toDegrees(targetAngle)-Math.toDegrees(angle)) < 5;
    }
}
