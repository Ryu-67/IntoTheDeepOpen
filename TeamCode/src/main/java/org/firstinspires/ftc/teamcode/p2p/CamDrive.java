package org.firstinspires.ftc.teamcode.p2p;

import android.graphics.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;

@Config
public class CamDrive {

    private PointRunner runner;
    private CameraSensor sensor;
    private SolvePNP solvePNP;


    public static double extendedOffset, retractedOffset;

    public CamDrive(PointRunner runner, CameraSensor sensor) {
        this.runner = runner; this.sensor = sensor;
    }

    private Translation2d translation2d = new Translation2d(0,0);
    private boolean flipWristFlag = false;

    public Pose targetOffsetSample(double sx, double sy) {
        translation2d = new Translation2d(sx, sy);
        translation2d = solvePNP.camXYToInchOffset(translation2d.getX(), translation2d.getY());

        return new Pose(translation2d.getX(), translation2d.getY());
    }

    public static class DetectSampleCommand extends CommandBase {

        CameraSensor camera;
        ElapsedTime timeout = new ElapsedTime();

        private boolean startFlag = true;

        public DetectSampleCommand(CameraSensor camera) {
            this.camera = camera;
        }

        @Override
        public void execute() {
            if (startFlag) {
                timeout.reset();
                startFlag = false;
            }
            camera.periodic();
        }

        @Override
        public boolean isFinished() {
            return timeout.seconds() > 1.5;
        }
    }

    private boolean started = false;
    private Pose startPose = new Pose(0, 0,0), target = new Pose(0, 0,0);

    public Pose convertCameraCoordsToCenteredOffset(double cx, double cy) {
        double corneredCy = 480-cy;
        double centerRelativeX = cx-320;
        double centerRelativeY = corneredCy-240;
        return targetOffsetSample(centerRelativeX, centerRelativeY);
    }

    public void setSampleTargetAlignment() {
        startPose = runner.getPose();
        target = convertCameraCoordsToCenteredOffset(sensor.getSamplePos().getX(), sensor.getSamplePos().getY());
        runner.setTarget(new Pose(startPose.getX()+target.getX(), startPose.getY()+target.getY()+retractedOffset, startPose.getHeading()));
    }

    public void update() {
        runner.update();
    }

    public boolean isDTaligned() {
        return runner.hasTargetBeenReached();
    }

}
