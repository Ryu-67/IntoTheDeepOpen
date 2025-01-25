package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class DriveFC {

    private DcMotor fl, fr, bl, br;

    private double x, y, z, d;
    private double gpx, gpy, gpz;

    private boolean isPr = false, downFlag = false, upFlag = false;

    private double mult = 1;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    private VoltageSensor voltageSensor; private boolean compEnabled = false;

    LazyImu imu;

    public DriveFC(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
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

        imu = new LazyImu(
                hardwareMap, "imu",
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );

        this.voltageSensor = voltageSensor; compEnabled = true;
    }

    public void setFlags(boolean down, boolean up) {
        upFlag = up; downFlag = down;
    }

    public void setXYZ(double x, double y, double z) {
        gpx = x; gpy = y; gpz = z;
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

        x = -gpy; y = -gpx; z = -gpz;
        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        applyGlobalPowers(imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), x * mult, y * mult, z * mult, d);
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
