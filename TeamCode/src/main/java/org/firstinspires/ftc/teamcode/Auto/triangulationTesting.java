package org.firstinspires.ftc.teamcode.Auto;

import org.apache.commons.math3.geometry.spherical.twod.Circle;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;
import org.opencv.core.*;


import java.util.ArrayList;
import java.util.List;

public class triangulationTesting extends OpenCvPipeline {

    private int width; // width of the image

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public triangulationTesting(int width) {
        this.width = width;
    }


    static final Scalar BLUE = new Scalar(0, 0, 255);




    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(230,90);
    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 50;



    Point region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region = new Mat();
    public boolean maxContourT = false;

    public double centerX;
    public double centerY;
    Mat mat = new Mat();

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */

        region = firstFrame.submat(new Rect(region_pointA, region_pointB));
        mat = firstFrame.submat(new Rect(region_pointA, region_pointB));
    }

    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        //Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        Point trial = new Point();
            trial.x = 200;
            trial.y = 200;
        Imgproc.rectangle(input, region_pointA, region_pointB, BLUE, 2);
        Imgproc.circle(input, trial , 1, new Scalar(0, 0, 255), 2);

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);
        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double maxArea = 0;
        MatOfPoint maxContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
                maxContourT = true;
            }
        }


        Point center = new Point();
        if (maxContour != null) {
            Moments moments = Imgproc.moments(maxContour);
            center.x = moments.get_m10() / moments.get_m00();
            center.y = moments.get_m01() / moments.get_m00();
            trial.x = moments.get_m10() / moments.get_m00();
            trial.y = moments.get_m01() / moments.get_m00();
            centerX = moments.get_m10() / moments.get_m00();
            centerY = moments.get_m01() / moments.get_m00();
            Imgproc.circle(input, center, 10, new Scalar(100, 240, 255), 2);
        }


         // return the mat with rectangles drawn
        return input;

    }


    static void drawRotatedRect(RotatedRect rect, Mat drawOn)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], BLUE, 2);
        }
    }


    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }
}

