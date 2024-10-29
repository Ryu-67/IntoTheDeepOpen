package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@Disabled
@Config
@TeleOp
public class MPPivotTuner extends OpMode {

    private MPPivot pivot;

    public static double target;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivot = new MPPivot(hardwareMap);
    }

    @Override
    public void loop() {
        pivot.setTargetAngle(target);
        pivot.update();
        telemetry.update();
    }
}