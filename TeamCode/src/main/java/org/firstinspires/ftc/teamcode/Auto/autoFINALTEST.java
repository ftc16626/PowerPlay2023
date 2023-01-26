package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class autoFINALTEST extends OpenCvPipeline {

    VideoCapture camera1 = new VideoCapture(0);
    VideoCapture camera2 = new VideoCapture(1);

    public int lastResult = 0;
    boolean test = false;


    public int objectPoints1 = 0;
    private int position = 3;


    public boolean left;
    public boolean right;
    //VisionTest vt = new VisionTest();
    //public Rect[] boundRect = new Rect[0];
    private int width;

    public autoFINALTEST(int width) {
        this.width = width;
    }



    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar lowerYellow = new Scalar(20, 100, 100);
    static final Scalar upperYellow = new Scalar(30, 255, 255);


    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(230,90);
    static final int REGION_WIDTH = 50;
    static final int REGION_HEIGHT = 50;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */

    Point region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region;
    int avgRed, avgGreen, avgBlue;

    // Volatile since accessed by OpMode thread w/o synchronization


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */



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
        Mat cameraMatrix1 = new Mat(3, 3, CvType.CV_64FC1);
        Mat cameraMatrix2 = new Mat(3, 3, CvType.CV_64FC1);

        Mat frame1 = new Mat();
        Mat frame2 = new Mat();

        Mat distCoeffs1 = new Mat();
        Mat distCoeffs2 = new Mat();
        // Rotation and translation vectors
        Mat R = new Mat();
        Mat T = new Mat();
        // Essential matrix
        Mat E = new Mat();
        // Fundamental matrix
        Mat F = new Mat();
        // Rectification transformation
        Mat R1 = new Mat();
        Mat R2 = new Mat();
        // Projection matrices
        Mat P1 = new Mat();
        Mat P2 = new Mat();
        // Disparity-to-depth mapping matrix
        Mat Q = new Mat();

        // 3D points

        // Convert the frames to the HSV color space
        Mat hsvImage1 = new Mat();
        Mat hsvImage2 = new Mat();
        Imgproc.cvtColor(frame1, hsvImage1, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(frame2, hsvImage2, Imgproc.COLOR_BGR2HSV);

        // Define a range of yellow colors in HSV

        // Threshold the frames to only select yellow colors
        Mat yellowMask1 = new Mat();
        Mat yellowMask2 = new Mat();
        Core.inRange(hsvImage1, lowerYellow, upperYellow, yellowMask1);
        Core.inRange(hsvImage2, lowerYellow, upperYellow, yellowMask2);


        // Find the contours of the yellow objects in both frames
        List<MatOfPoint> contours1 = new ArrayList<>();
        List<MatOfPoint> contours2 = new ArrayList<>();
        Imgproc.findContours(yellowMask1, contours1, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowMask2, contours2, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Distortion coefficients


    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
//        inputToCb(input);
//        Mat mat = new Mat();
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */

        // Find the largest contour in both frames
        MatOfPoint3f objectPoints = new MatOfPoint3f();
        double maxArea1 = 0;
        double maxArea2 = 0;
        MatOfPoint largestContour1 = null;
        MatOfPoint largestContour2 = null;
        for (MatOfPoint contour : contours1) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea1) {
                maxArea1 = area;
                largestContour1 = contour;
            }
        }
        for (MatOfPoint contour : contours2) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea2) {
                maxArea2 = area;
                largestContour2 = contour;
            }
        }

        Moments moments1 = Imgproc.moments(largestContour1);
        Moments moments2 = Imgproc.moments(largestContour2);
        int x1 = (int) (moments1.get_m10() / moments1.get_m00());
        int y1 = (int) (moments1.get_m01() / moments1.get_m00());
        int x2 = (int) (moments2.get_m10() / moments2.get_m00());
        int y2 = (int) (moments2.get_m01() / moments2.get_m00());
        //System.out.println("Position from camera 1: (" + x1 + ", " + y1 + ")");
        telemetry.addData("Position from camera 1: (", + x1 + ", ", + y1 + ")");
        telemetry.addData("Position from camera 2: (", + x2 + ", ", + y2 + ")");
        //System.out.println("Position from camera 2: (" + x2 + ", " + y2 + ")");

        // Create a vector of 2D points
        MatOfPoint2f imagePoints1 = new MatOfPoint2f(new Point(x1, y1));
        MatOfPoint2f imagePoints2 = new MatOfPoint2f(new Point(x2, y2));

        // Triangulate the position of the object
        Calib3d.triangulatePoints(P1, P2, imagePoints1, imagePoints2, objectPoints);

        // Print the 3D position of the object
        //System.out.println("3D Position of the object: " + objectPoints.toString());
        return input;


    }
/*
    @Override
    public Mat processFrame(Mat input) {
        /*
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) return input;
        Scalar lowHSV = new Scalar(20, 100, 100);
        Scalar highHSV = new Scalar(30, 255, 255);
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);
        thresh.release();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        edges.release();
        hierarchy.release();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
       Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        left = false;
        right = false;
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x) left = true;
            if (boundRect[i].x + boundRect[i].width > right_x) right = true;
            //if(boundRect[i].x > 0) test = true;
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }
        lastResult = 1;
        return mat;
    }
    */



    public int getLastResult() {
        return lastResult;
    }

    public int getObjectPoints() { return objectPoints;}

    public int getCameraPosition() { return x1;}

    public int getPosition() {
        return position;
    }

    public boolean getRight() {
        return right;
    }

    public boolean getTest() {
        return test;
    }

    public int getAvgRed(){
        return avgRed;
    }
    public int getAvgGreen(){
        return avgGreen;
    }
    public int getAvgBlue(){
        return avgBlue;
    }
}
