package org.firstinspires.ftc.teamcode.p2p;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;

public class VisionDTAlignCommand extends CommandBase {

    CamDrive drive;

    public VisionDTAlignCommand(PointRunner runner, CameraSensor sensor) {
        drive = new CamDrive(runner, sensor);
    }

    private boolean startFlag = true;

    @Override
    public void execute() {
        if (startFlag) {
            drive.setSampleTargetAlignment();
            startFlag = false;
        }

        drive.update();
    }

    @Override
    public boolean isFinished() {
        if (startFlag) {
            return false;
        } else {
            return drive.isDTaligned();
        }
    }

}
