package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
@Config
public class LiftTuner extends OpMode {

    private Lift lift;
    public static double target = 0;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        lift.setTarget(target);
        lift.update();

        telemetry.addData("ticks", lift.getTicks());
        telemetry.update();
    }
}
