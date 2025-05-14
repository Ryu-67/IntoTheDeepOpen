package org.firstinspires.ftc.teamcode.ocv;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
public class SolvePNP {
    double cx = 319.495, cy = 242.502, fl = 822.317;
    public static double rx = 0;
    public static double ry = 0;
    public static double rz = 0;
    public static double rp = Math.toRadians(-90); //robot xyz, robot pitch
    Pose2d fieldPos;

    private double cxi = 1.0/90.0, cyi = 1.0/94.0;

    public Translation2d getRobotCentricTranslation(double px, double py) {
        double gt = Math.atan((py-cy)/fl);
        Log.v("camera centric angle", gt + "");
        double theta = rp-gt;
        Log.v("ground centric angle", theta + "");
        double gd = rz / Math.tan(theta);
        Log.v("ground distance", gd + "");
        double dist = Math.hypot(gd, rz);
        Log.v("sample distance", dist + "");
        double pd = Math.cos(gt)*dist;
        Log.v("plane distance", pd + "");
        double gx = (px-cx)*pd/fl;
        Log.v("ground horizontal distance", gx + "");
        // gd is backwards for some reason
        return new Translation2d(-gd + rx, ry-gx);
    }

    public Translation2d camXYToInchOffset(double cX, double cY) {
        return new Translation2d(cX*cxi, cY*cyi);
    }

    public Vector2d getFieldCoordinates(double px, double py, Pose2d robotPose) {

        Translation2d rctrans = getRobotCentricTranslation(px, py);

        Vector2d xBasis = new Vector2d(1, 0).rotateBy(robotPose.getRotation().getDegrees());
        Vector2d yBasis = new Vector2d(0, 1).rotateBy(robotPose.getRotation().getDegrees());

        Vector2d robotPos = new Vector2d(robotPose.getX(), robotPose.getY());

        return robotPos.plus(xBasis.scale(rctrans.getX())).plus(yBasis.scale(rctrans.getY()));

    }

    public void setScaleFactors(double xScale, double yScale) {
        this.cyi = yScale; this.cxi = xScale;
    }

}
