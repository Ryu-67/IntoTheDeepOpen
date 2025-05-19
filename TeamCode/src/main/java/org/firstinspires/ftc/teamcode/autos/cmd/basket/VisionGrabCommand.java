package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@Config
public class VisionGrabCommand extends CommandBase {

    CameraSensor sensor; SolvePNP solvePNP;
    Deposit deposit; MPPivot pivot; PinpointDrive drive;

    double wrist = 0; boolean rLoop = true; ElapsedTime elapsedTime = new ElapsedTime();

    public static double visionTimeout = 1.2;

    private enum State {
        scanning,
        shifting,
        calculating,
        grabbing,
        finished
    }

    MecanumDrive.Params params;  Pose2d correctedOffset, offset;

    State state = State.scanning;
    Telemetry t;

    public VisionGrabCommand(CameraSensor sensor, Deposit deposit, MPPivot pivot, PinpointDrive drive,
                             double wrist, Telemetry t) {
        this.sensor = sensor; this.deposit = deposit; this.pivot = pivot; this.drive = drive;
        this.wrist = wrist; solvePNP = new SolvePNP();
        params = new MecanumDrive.Params();
        this.t = t;
    }

    Action a;

    @Override
    public void execute() {
        if (rLoop) {
            rLoop = false;
            elapsedTime.reset();
        }

        t.clearAll();

        switch (state) {
            case scanning:
                sensor.periodic();
                t.addLine("scanning");
                if (elapsedTime.seconds() >= visionTimeout && sensor.detections()) {
                    state = State.calculating;
                    t.addLine("found");
                } else if (elapsedTime.seconds() >= visionTimeout && !sensor.detections()) {
                    state = State.shifting;
                    t.addLine("cooked");
                }
                break;
            case calculating:
                Pose2d current = drive.pose;
                t.addLine("posturing");
                Vector2d sample = sensor.getSamplePos();
                offset = convertCameraCoordsToCenteredOffset(sample.getX(), sample.getY());
                correctedOffset = new Pose2d(offset.position.y-0.7, -offset.position.x, Math.toRadians(0));
                correctedOffset = new Pose2d(correctedOffset.position.x+current.position.x, correctedOffset.position.y+current.position.y, 0);
                deposit.setDeposit(wrist, Deposit.aPitchBack, Deposit.armBack, Claw.open);

                MecanumDrive.PARAMS.positionalError = 0.2;
                MecanumDrive.PARAMS.lateralVelGain = 25;
                MecanumDrive.PARAMS.lateralGain = 30;
                MecanumDrive.PARAMS.timeout = 0.1;
                a = drive.actionBuilder(current)
                        .setTangent(0)
                        .lineToX(correctedOffset.position.x)
                        .setTangent(Math.toRadians(Math.signum(correctedOffset.position.y) * 90))
                        .lineToY(correctedOffset.position.y).build();
                pivot.setTargetAngle(0);
                state = State.grabbing;
                break;
            case grabbing:
                t.addLine("SHOULD be grabbing rn");
                if (!a.run(new TelemetryPacket())) {
                    deposit.setDeposit(wrist, Deposit.aPitchBack, Deposit.armBack, Claw.closed);
                    state = State.finished;
                    MecanumDrive.PARAMS = params;
                }
                break;
        }
        t.update();
    }

    @Override
    public boolean isFinished() {
        return state == State.finished;
    }

    public Pose2d targetOffsetSample(double sx, double sy) {
        Translation2d translation2d = new Translation2d(sx, sy);
        translation2d = solvePNP.camXYToInchOffset(translation2d.getX(), translation2d.getY());

        return new Pose2d(translation2d.getX(), translation2d.getY(), 0);
    }

    public Pose2d convertCameraCoordsToCenteredOffset(double cx, double cy) {
        double corneredCy = 480-cy;
        double centerRelativeX = cx-320;
        double centerRelativeY = corneredCy-240;
        return targetOffsetSample(centerRelativeX, centerRelativeY);
    }

}
