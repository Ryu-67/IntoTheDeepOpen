package org.firstinspires.ftc.teamcode.p2p;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.p2p.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Config
public class PointRunner {

    private DcMotor fl, fr, bl, br;
    private PinpointLocalizer localizer;

    private double x, y, z;

    private double headingError, xError, yError;

    private PSQUIDController xPID, yPID;
    private PIDController hPID, sXPID, sYPID, sHPID;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public PointRunner(HardwareMap hardwareMap) {
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

        localizer = new PinpointLocalizer(hardwareMap, new Pose(0,0,0));

        xPID = new PSQUIDController(px); yPID = new PSQUIDController(py); hPID = new PIDController(ph, 0,0);
        sXPID = new PIDController(spx, 0, 0); sYPID = new PIDController(spy, 0, 0); sHPID = new PIDController(sph, 0, 0);

    }

    public static double px, py, ph;
    public static double spx, spy, sph;

    public static double translationalSecondaryThreshold = 2, headingSecondaryThreshold = 10;

    private Pose target = new Pose(0, 0, 0);
    private Pose pose = new Pose(0, 0,0);

    private double flpower = 0, frpower = 0, blpower = 0, brpower = 0;

    public void update() {
        localizer.update();
        pose = localizer.getPose();

        headingError = Math.IEEEremainder(target.getHeading() - pose.getHeading(), 2 * Math.PI);
        xError = target.getX() - pose.getX();
        yError = target.getY() - pose.getY();

        xPID.setP(px); yPID.setP(py); hPID.setP(ph);
        sXPID.setP(spx); sYPID.setP(spy); sHPID.setP(sph);

        double translationalError = Math.hypot(xError, yError);

        if (Math.abs(translationalError) < translationalSecondaryThreshold) {

            x = sXPID.calculate(xError);
            y = sYPID.calculate(yError);

            double x_rotated = x * Math.cos(pose.getHeading()) - y * Math.sin(pose.getHeading());
            double y_rotated = x * Math.sin(pose.getHeading()) + y * Math.cos(pose.getHeading());

            flpower = (x_rotated + y_rotated);
            blpower = (x_rotated - y_rotated);
            frpower = (x_rotated - y_rotated);
            brpower = (x_rotated + y_rotated);
        } else {
            x = xPID.compute(xError);
            y = yPID.compute(yError);

            double x_rotated = x * Math.cos(pose.getHeading()) - y * Math.sin(pose.getHeading());
            double y_rotated = x * Math.sin(pose.getHeading()) + y * Math.cos(pose.getHeading());

            flpower = (x_rotated + y_rotated);
            blpower = (x_rotated - y_rotated);
            frpower = (x_rotated - y_rotated);
            brpower = (x_rotated + y_rotated);
        }

        if (Math.toDegrees(Math.abs(headingError)) < 10) {
            z = sHPID.calculate(headingError);
        } else {
            z = hPID.calculate(headingError);
        }
        flpower += z;
        blpower += z;
        frpower -= z;
        brpower -= z;

        fl.setPower(flpower);
        fr.setPower(frpower);
        bl.setPower(blpower);
        br.setPower(brpower);

    }

    public Pose getTarget() {
        return target;
    }

    public void setTarget(Pose target) {
        this.target = target;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
        localizer.setPose(pose);
    }

    public Pose getPose() {
        return pose;
    }

    public boolean hasTargetBeenReached() {
        return
                Math.abs(xError) < 0.15 && Math.abs(yError) < 0.15 && Math.abs(headingError) < Math.toRadians(2);
    }
}
