package pedroPathing.startcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class DetectionR extends ColorDetectionPipeline {
    private static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.75;  // Actual object width
    private static final double FOCAL_LENGTH = 728;  // Focal length in pixels
    private static final double MIN_CONTOUR_AREA = 500.0;  // Minimum contour area to consider

    private Mat hsvFrame = new Mat();
    private Mat yellowMask = new Mat();
    private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private Mat hierarchy = new Mat();
    private Mat latestMat = new Mat();

    private double cX = 0;
    private double cY = 0;
    private double width = 300;
    private int numberOfFramesProcessed = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Threshold for yellow color
        Scalar lowerYellow = new Scalar(100, 100, 100);
        Scalar upperYellow = new Scalar(180, 255, 255);
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        // Morphological transformations to clean up noise
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw contour and calculate centroid
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 0, 255), 2);

            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            // Calculate bounding box width
            width = Imgproc.boundingRect(largestContour).width;

            // Display information
            String widthLabel = "Width: " + (int) width + " pixels";
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance()) + " inches";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        }

        // Update frame count and latest processed frame
        numberOfFramesProcessed++;
        input.copyTo(latestMat);
        return input;
    }
    public double getCenterX() {
        return cX;
    }

    public double getCenterY() {
        return cY;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }

    public double getDistance() {
        return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * FOCAL_LENGTH) / width;
    }

    public double getWidth() {
        return width;
    }

    public Mat getLatestMat() {
        return latestMat;
    }

    @Override
    public int framesTotal() {
        return numberOfFramesProcessed;
    }
}
