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

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public Drive(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("backLeft");
        bl = hardwareMap.dcMotor.get("frontLeft");
        fr = hardwareMap.dcMotor.get("backRight");
        br = hardwareMap.dcMotor.get("frontRight");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        fr .setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private VoltageSensor voltageSensor; private boolean compEnabled = false;

    public Drive(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        fl = hardwareMap.dcMotor.get("frontLeft");
        bl = hardwareMap.dcMotor.get("backLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        br = hardwareMap.dcMotor.get("backRight");

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
        gpx =x; gpy = y; gpz = z;
    }

    public void setInputs(double x, double y, double z) {
        this.x = x; this.y = y; this.z = z;
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

    public void updateAuto(double heading, double x, double y, double z) {
        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        applyGlobalPowers(heading, x, y, z, d);
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

    public void applyGlobalPowers(double heading, double x, double y, double z, double d) {
        double comp = Robot.universalVoltage/voltageSensor.getVoltage();

        x = -x; y = y; z = -z;

        double xr = x * Math.cos(heading) - y * Math.sin(heading);
        double yr = x * Math.sin(heading) + y * Math.cos(heading);

        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        fl.setPower(((xr+yr-z)/d) * comp);
        bl.setPower(((xr-yr-z)/d) * comp);
        fr.setPower(((xr-yr+z)/d) * comp);
        br.setPower(((xr+yr+z)/d) * comp);
    }

}
