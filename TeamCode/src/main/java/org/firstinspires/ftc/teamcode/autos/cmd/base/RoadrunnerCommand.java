package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class RoadrunnerCommand extends CommandBase {

    private Action follow;
    private boolean finished = false;

    public RoadrunnerCommand(Action follow) {
        this.follow = follow;
    }


    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        follow.preview(packet.fieldOverlay());
        finished = !follow.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
