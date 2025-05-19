package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import org.firstinspires.ftc.teamcode.ocv.ColorRange;
import org.firstinspires.ftc.teamcode.ocv.GlowUpPipeline;
import org.firstinspires.ftc.teamcode.ocv.ImageRegion;
import org.firstinspires.ftc.teamcode.ocv.SampleLocatorPipeline;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class CameraSensor extends SubsystemBase {
    public static Scalar minimumRed1 = new Scalar(0, 0, 0);
    public static Scalar maximumRed1 = new Scalar(12, 0, 0);

    public static Scalar minimumRed2 = new Scalar(0, 0, 0);
    public static Scalar maximumRed2 = new Scalar(0, 0, 0);

    public static Scalar minimumBlue = new Scalar(0, 0, 0);
    public static Scalar maximumBlue = new Scalar(0, 0, 0);

    public static Scalar minimumYellow = new Scalar(12, 70, 70);
    public static Scalar maximumYellow = new Scalar(50, 180, 180);

    public static boolean useGlowUp = false;

    public static int exposureMillis = 30;
    public static int minContourArea = 880, maxCountourArea = 100000;
    public static double alpha = 0.5; //gain scalar
    public static double beta = 0; //brightness offset
    public static int contrast = 30; //default is 40


    SampleLocatorPipeline colorLocator;
    GlowUpPipeline glowUp;
    private double pixelPos = 0;

    private Vector2d samplePos = new Vector2d();

    Telemetry telemetry;
    VisionPortal visionPortal;

    ColorBlobLocatorProcessor.Blob firstDetection;
    WebcamName camera;


    public CameraSensor(HardwareMap hMap, String name, Telemetry telemetry, int viewportid, boolean red, boolean drawContours){
        glowUp = new GlowUpPipeline(alpha, beta, contrast);
        colorLocator = new SampleLocatorPipeline(
                new org.firstinspires.ftc.teamcode.ocv.ColorRange(ColorSpace.HSV, new Scalar(13, 60, 60), new Scalar(50, 255, 255)),
                ImageRegion.asImageCoordinates(0, 0, 1280, 720),
                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY,
                -1,
                -1,
                drawContours,
                -1,
                Color.rgb(255, 120, 31),
                Color.rgb(255, 255, 255),
                Color.rgb(3, 227, 252)
        );

        // add yellow colors (same for all alliances)
        colorLocator.addColors(new org.firstinspires.ftc.teamcode.ocv.ColorRange(ColorSpace.HSV, minimumYellow, maximumYellow));

        if (red) {
                colorLocator.addColors(new org.firstinspires.ftc.teamcode.ocv.ColorRange(ColorSpace.HSV, minimumRed1, maximumRed1));
                colorLocator.addColors(new org.firstinspires.ftc.teamcode.ocv.ColorRange(ColorSpace.HSV, minimumRed2, maximumRed2));
        } else {
                colorLocator.addColors(new ColorRange(ColorSpace.HSV, minimumBlue, maximumBlue));
        }

        /*int viewportid = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)[0];
        if (name.equals(VisionConstants.slideCameraName)) {
            Log.i("Viewport", "is this working");
            viewportid = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)[1];
        }*/
        if (useGlowUp) {
            visionPortal = new VisionPortal.Builder()
                    .addProcessors(glowUp, colorLocator)
                    .setCameraResolution(new Size(640, 480))
                    .setCamera(hMap.get(WebcamName.class, name))
                    .enableLiveView(true)
                    .setAutoStartStreamOnBuild(true)
//                    .setLiveViewContainerId(viewportid)
                    .build();
        }

        camera = hMap.get(WebcamName.class, name);



        this.telemetry = telemetry;
    }

    public boolean sensed = false;

    public void startStreaming() {
        visionPortal = new VisionPortal.Builder()
                .addProcessors(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(camera)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
//                    .setLiveViewContainerId(viewportid)
                .build();

        waitForSetExposure(1000, 1000);
        visionPortal.setProcessorEnabled(colorLocator, false);
    }

    public void enableProc () {
        visionPortal.setProcessorEnabled(colorLocator, true);
    }

    @Override
    public void periodic() {
        pixelPos = 0;

        if (visionPortal.getProcessorEnabled(colorLocator)) {

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(minContourArea, maxCountourArea, blobs);
            int dist = 10000;
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

            if (!blobs.isEmpty()) {
                /*for (int i = 0; i < Math.min(blobs.size(), 3); i++) {
                    if (Math.abs(160 - blobs.get(i).getBoxFit().center.x) < Math.abs(dist)) {
                        dist = (int) (160 - blobs.get(i).getBoxFit().center.x);
                    }
                }

                pixelPos = dist;*/
                sensed = true;
                pixelPos = (int) (160 - blobs.get(0).getBoxFit().center.x);
                samplePos = new Vector2d(blobs.get(0).getBoxFit().center.x, blobs.get(0).getBoxFit().center.y);
            } else {
                sensed = false;
            }
        }
    }

    public boolean detections() {
        return sensed;
    }

    public Vector2d getSamplePos() {
        return samplePos;
    }



    public void setEnabled(boolean enable) {
        visionPortal.setProcessorEnabled(colorLocator, enable);
    }

    public double getPixelPos() {
        return pixelPos;
    }

    /**
     * @return whether the set was successful or not
     */
    public boolean setExposure(int exposure) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        ExposureControl control = visionPortal.getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        Log.i("camera", "exposure: " + control.getExposure(TimeUnit.MILLISECONDS));
        return control.setExposure(exposure, TimeUnit.MILLISECONDS);
    }
    public boolean setExposure() {
        return setExposure(exposureMillis);
    }
    public boolean waitForSetExposure(long timeoutMs, int maxAttempts) {
        return waitForSetExposure(timeoutMs, maxAttempts, exposureMillis);
    }

    public void setOnlyYellow(boolean onlyYellow) {
        colorLocator.onlyFirstColor = onlyYellow;
    }

    public boolean waitForSetExposure(long timeoutMs, int maxAttempts, int exposure) {
        long startMs = System.currentTimeMillis();
        int attempts = 0;
        long msAfterStart = 0;
        while (msAfterStart < timeoutMs && attempts++ < maxAttempts) {
            Log.i("camera", String.format("Attempting to set camera exposure, attempt %d, %d ms after start", attempts, msAfterStart));
            if (setExposure(exposure)) {
                Log.i("camera", "Set exposure succeeded");
                return true;
            }
            msAfterStart = System.currentTimeMillis() - startMs;
        }

        Log.e("camera", "Set exposure failed");
        return false;
    }
    public void saveFrame(String name) {
        visionPortal.saveNextFrameRaw(name);
    }

    public double readSampleAngle() {
        return colorLocator.getBlobs().get(0).getBoxFit().angle;
    }
}