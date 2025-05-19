package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

public class RRFollowCommand extends CommandBase {

    private Action action;
    private double timeout = 0.2;

    public RRFollowCommand(Action action, double timeout) {
        this.action = action;
        this.timeout = timeout;
    }

    public boolean flagged = false;

    @Override
    public boolean isFinished() {
        if (!flagged) {
            MecanumDrive.PARAMS.timeout = timeout;
            flagged = true;
        }
        return !action.run(new TelemetryPacket());
    }

}
