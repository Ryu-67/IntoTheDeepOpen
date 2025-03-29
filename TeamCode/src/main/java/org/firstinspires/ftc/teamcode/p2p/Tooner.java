package org.firstinspires.ftc.teamcode.p2p;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
@TeleOp(name = "SQUID tuner")
public class Tooner extends OpMode {

    public PointRunner runner;
    public static double x, y, h;

    public Pose current;

    @Override
    public void init() {
        runner = new PointRunner(hardwareMap);
    }

    @Override
    public void loop() {
        runner.setTarget(new Pose(x, y, Math.toRadians(h)));
        runner.update();
        current = runner.getPose();
        telemetry.addData("Current X", current.getX());
        telemetry.addData("Current Y", current.getY());
        telemetry.addData("Current Heading", Math.toDegrees(current.getHeading()));
        telemetry.update();
    }
}
