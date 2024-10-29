package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.Utils;

@Config
public class MPPivot {

    private DcMotor lPivot, rPivot;

    private RevTouchSensor reset;

    PIDController controller = new PIDController(p, i, d);
    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2*Math.PI, 8*Math.PI);
    public static double p = 3, i = 0, d = 0.1, k = 0.2;

    public double tpr = (537.7*(80/30))/(2*Math.PI);
    public boolean flag = false;

    public static double vertAngle = 117, backUpAngle = 130, specimen = 70;

    public double updateCurrentAngle(double current) {
        if (reset.isPressed()) {
            rPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flag = true;
            return  0;
        } else {
            flag = false;
            return  current/tpr;
        }
    }

    double angle = 0, langle = angle, aVelocity = 0;

    public MPPivot(HardwareMap hardwareMap) {
        rPivot = hardwareMap.dcMotor.get("rPivot");
        lPivot = hardwareMap.dcMotor.get("lPivot");

        rPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        lPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset = hardwareMap.get(RevTouchSensor.class, "reset");

        profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
    }

    public double targetAngle = 0, power = 0, lastPower = power, lta = targetAngle, indexedPosition = 0;

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = Math.toRadians(targetAngle);
    }

    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    public void update() {

        angle = Math.abs(updateCurrentAngle(rPivot.getCurrentPosition()));

        if (targetAngle != lta) {
            profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(targetAngle, 0), new TrapezoidProfile.State(angle, aVelocity));
            fullTimer.reset();
        }

        indexedPosition = profile.calculate(fullTimer.seconds()).position;

        controller.setSetPoint(indexedPosition);

        power = controller.calculate(angle) + (k * Math.cos(targetAngle));

        if (flag && Utils.valInThresh(0, lastPower, 0) && targetAngle == 0) {
            apply(0);
        } else if (Utils.valInThresh(power, lastPower, 0.01)) {
            apply(power);
            lastPower = power;
        }

        lta = targetAngle;

        aVelocity = (angle-langle)/velTimer.seconds();
        langle = angle;
        velTimer.reset();
    }

    public void apply(double p) {
        rPivot.setPower(-p);
        lPivot.setPower(-p);
    }

    public boolean check() {
        return Math.abs(Math.toDegrees(targetAngle)-Math.toDegrees(angle)) < 5;
    }
}
