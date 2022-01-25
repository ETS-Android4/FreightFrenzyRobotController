package org.firstinspires.ftc.teamcode.util.misc.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVElementTracker {

    // TODO: Use openCameraDeviceAsync

    public enum LOCATION {
        LEFT,
        RIGHT,
        UNKNOWN
    }

    OpenCvCamera camera;
    TeamElementPipeline pipeline;

    public OpenCVElementTracker(HardwareMap hw) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        pipeline = new TeamElementPipeline();
        camera.setPipeline(pipeline);
        stream();
    }

    public void stream() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(camera, 30);
    }

    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }

    public LOCATION getLocation() {
        return pipeline.getLocation();
    }

}

@Config
class TeamElementPipeline extends OpenCvPipeline {

    // TODO: Add a constructor that takes in an offset to shift the x position of the mats

    // https://www.youtube.com/watch?v=JO7dqzJi8lw

    public static double BLUE_THRESHOLD = 0.25;

    public static double lroix1 = 20;
    public static double lroiy1 = 120;
    public static double lroix2 = 140;
    public static double lroiy2 = 200;
    public static double rroix1 = 180;
    public static double rroiy1 = 120;
    public static double rroix2 = 300;
    public static double rroiy2 = 200;

    Mat mat = new Mat();
    static Rect LEFT_ROI = new Rect(
            new Point(lroix1, lroiy1),
            new Point(lroix2, lroiy2)
    );

    static Rect RIGHT_ROI =  new Rect(
        new Point(rroix1, rroiy1),
        new Point(rroix2, rroiy2)
    );

    OpenCVElementTracker.LOCATION currentLoc = OpenCVElementTracker.LOCATION.UNKNOWN;

    public static boolean showGrayscale = false;

    @Override
    public Mat processFrame(Mat input) {

        // Convert to grayscale image of blue values
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowBlue = new Scalar(20, 50, 70);
        Scalar highBlue = new Scalar(40, 255, 255);
        Core.inRange(mat, lowBlue, highBlue, mat);

        // Get the average amount of blue in each region of interest
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        double leftValue = Core.mean(left).val[0];
        double rightValue = Core.mean(right).val[0];
        left.release();
        right.release();

        // Find the region with the most blue
        if ((leftValue > BLUE_THRESHOLD) && (leftValue > rightValue)) {
            currentLoc = OpenCVElementTracker.LOCATION.LEFT;
        } else if ((rightValue > BLUE_THRESHOLD) && (rightValue > leftValue)) {
            currentLoc = OpenCVElementTracker.LOCATION.RIGHT;
        } else {
            currentLoc = OpenCVElementTracker.LOCATION.UNKNOWN;
        }

        // Draw ROI rectangles to the screen and highlight the one with the element
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        if (!showGrayscale) {
            input.copyTo(mat);
        }
        Scalar emptyColor = new Scalar(255, 0, 0);
        Scalar foundColor = new Scalar(0, 255, 0);
        Imgproc.rectangle(mat, LEFT_ROI, (currentLoc == OpenCVElementTracker.LOCATION.LEFT) ? foundColor : emptyColor);
        Imgproc.rectangle(mat, RIGHT_ROI, (currentLoc == OpenCVElementTracker.LOCATION.RIGHT) ? foundColor : emptyColor);

        return mat;
    }

    @Override
    public void onViewportTapped() {
        super.onViewportTapped();
        showGrayscale = !showGrayscale;
    }

    public OpenCVElementTracker.LOCATION getLocation() {
        return currentLoc;
    }

}