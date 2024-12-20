package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class RoadrunnerCommand extends CommandBase {

    private Action follow;

    public RoadrunnerCommand(Action follow) {
        this.follow = follow;
    }

    @Override
    public boolean isFinished() {
        return !follow.run(new TelemetryPacket());
    }

}
