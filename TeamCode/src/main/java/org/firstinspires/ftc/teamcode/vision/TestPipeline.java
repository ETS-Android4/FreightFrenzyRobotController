package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {

    // https://www.youtube.com/watch?v=JO7dqzJi8lw

    enum LOCATION {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    Mat mat = new Mat();
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 120),
            new Point(100, 200)
    );
    static final Rect MID_ROI = new Rect(
            new Point(140, 120),
            new Point(180, 200)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(220, 120),
            new Point(260, 200)
    );

    LOCATION currentLoc = LOCATION.UNKNOWN;

    boolean showGrayscale = false;

    @Override
    public Mat processFrame(Mat input) {
        // Convert to grayscale image of blue values
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowBlue = new Scalar(110, 50, 70);
        Scalar highBlue = new Scalar(130, 255, 255);
        Core.inRange(mat, lowBlue, highBlue, mat);

        // Get the average amount of blue in each region of interest
        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        double leftValue = Core.mean(left).val[0];
        double midValue = Core.mean(mid).val[0];
        double rightValue = Core.mean(right).val[0];
        left.release();
        mid.release();
        right.release();

        // Find the region with the most blue
        if ((leftValue > midValue) && (leftValue > rightValue)) {
            currentLoc = LOCATION.LEFT;
        } else if ((midValue > leftValue) && (midValue > rightValue)) {
            currentLoc = LOCATION.MIDDLE;
        } else if ((rightValue > leftValue) && (rightValue > midValue)) {
            currentLoc = LOCATION.RIGHT;
        } else {
            currentLoc = LOCATION.UNKNOWN;
        }

        // Draw ROI rectangles to the screen and highlight the one with the element
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        if (!showGrayscale) {
            input.copyTo(mat);
        }
        Scalar emptyColor = new Scalar(255, 0, 0);
        Scalar foundColor = new Scalar(0, 255, 0);
        Imgproc.rectangle(mat, LEFT_ROI, (currentLoc == LOCATION.LEFT) ? foundColor : emptyColor);
        Imgproc.rectangle(mat, MID_ROI, (currentLoc == LOCATION.MIDDLE) ? foundColor : emptyColor);
        Imgproc.rectangle(mat, RIGHT_ROI, (currentLoc == LOCATION.RIGHT) ? foundColor : emptyColor);

        return mat;
    }

    @Override
    public void onViewportTapped() {
        super.onViewportTapped();
        showGrayscale = !showGrayscale;
    }

    public LOCATION getLocation() {
        return currentLoc;
    }

}