package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@TeleOp
@Config
public class PosRecorder extends OpMode {

    Lift lift; MPPivot pivot; Deposit deposit;

    public static double pAngle = 0, liftSet = 0, arm = 0.4, pitch = 0.29, claw = 0, wrist = 9;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, false); pivot = new MPPivot(hardwareMap);
        deposit = new Deposit(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        lift.setTarget(liftSet);
        pivot.setTargetAngle(pAngle);
        deposit.setDeposit(wrist, pitch, arm, claw);
        lift.update();
        pivot.update();
    }
}
