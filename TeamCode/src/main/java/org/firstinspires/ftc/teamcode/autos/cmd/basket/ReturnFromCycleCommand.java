package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

@Config
public class ReturnFromCycleCommand extends CommandBase {
    private Action action;
    private double timeout = 0.2;
    private PinpointDrive drive;

    public static Pose2d returnPose = new Pose2d(-52, -64, Math.toRadians(45));

    public ReturnFromCycleCommand(PinpointDrive drive, double timeout) {
        this.drive = drive;
        this.timeout = timeout;
    }

    public boolean flagged = false;

    @Override
    public boolean isFinished() {
        if (!flagged) {
            MecanumDrive.PARAMS.timeout = timeout;
            action = drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(returnPose, Math.toRadians(-138))
                    .build();
            flagged = true;
        }
        return !action.run(new TelemetryPacket());
    }

}
