package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
@Config
@Disabled
public class LiftTuner extends OpMode {

    private Lift lift;
    public static double target = 0;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, false);
    }

    @Override
    public void loop() {
        lift.setTarget(target);
        lift.update();

        telemetry.addData("ticks", lift.getTicks());
        telemetry.update();
    }
}
