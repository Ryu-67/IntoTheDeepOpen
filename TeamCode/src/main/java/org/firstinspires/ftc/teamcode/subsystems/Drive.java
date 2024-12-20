package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Drive {

    private DcMotor fl, fr, bl, br;

    private double x, y, z, d;
    private double gpx, gpy, gpz;

    private boolean isPr = false, downFlag = false, upFlag = false;

    private double mult = 0.8;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

    public Drive(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("backLeft");
        bl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("backRight");
        br = hardwareMap.dcMotor.get("frontRight");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        fr .setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private VoltageSensor voltageSensor; private boolean compEnabled = false;

    public Drive(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        fl = hardwareMap.dcMotor.get("backLeft");
        bl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("backRight");
        br = hardwareMap.dcMotor.get("frontRight");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        fr .setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.voltageSensor = voltageSensor; compEnabled = true;
    }

    public void setFlags(boolean down, boolean up) {
        upFlag = up; downFlag = down;
    }

    public void setXYZ(double x, double y, double z) {
        gpx =x; gpy = -y; gpz = z;
    }

    public void update() {
        if (downFlag && !isPr && mult > 0.1) {
            mult -= 0.1;
            isPr = true;
        } else if (upFlag && !isPr && mult < 1) {
            mult += 0.1;
            isPr = true;
        } else if (!downFlag && !upFlag) {
            isPr = false;
        }

        x = -gpx; y = gpy; z = gpz;
        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        smp(x * mult, y * mult, z * mult);
    }

    public double getMultiplier() {
        return mult;
    }

    private double abs(double i) {
        return Math.abs(i);
    }

    public void smp(double x, double y, double z) {
        if (compEnabled) {
            double comp = Robot.universalVoltage/voltageSensor.getVoltage();
            fl.setPower(((y + x - z)/d) * comp);
            bl.setPower(((y - x - z)/d) * comp);
            fr.setPower(((y - x + z)/d) * comp);
            br.setPower(((y + x + z)/d) * comp);
        } else {
            fl.setPower((y + x - z)/d);
            bl.setPower((y - x - z)/d);
            fr.setPower((y - x + z)/d);
            br.setPower((y + x + z)/d);
        }
    }

}
