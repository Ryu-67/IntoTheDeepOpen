package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
@Config
public class HangTuner extends OpMode {

    Lift lift;

    public static double power = 0;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, false);
        lift.setRunMode(Lift.Mode.hang);
    }

    @Override
    public void loop() {
        lift.update();
    }

}
