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

public class triangulationTest extends OpenCvPipeline {
    enum junctionLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width; // width of the image
    junctionLocation location;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public triangulationTest(int width) {
        this.width = width;
    }
    VideoCapture camera1 = new VideoCapture(0);
    VideoCapture camera2 = new VideoCapture(1);



    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat cameraMatrix1 = new Mat(3, 3, CvType.CV_64FC1);
        Mat cameraMatrix2 = new Mat(3, 3, CvType.CV_64FC1);
        // Distortion coefficients
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
        MatOfPoint3f objectPoints = new MatOfPoint3f();

        Mat frame1 = new Mat();
        Mat frame2 = new Mat();
        camera1.read(frame1);
        camera2.read(frame2);

        // Convert the frames to the HSV color space
        Mat hsvImage1 = new Mat();
        Mat hsvImage2 = new Mat();
        Imgproc.cvtColor(frame1, hsvImage1, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(frame2, hsvImage2, Imgproc.COLOR_BGR2HSV);

        // Define a range of yellow colors in HSV
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

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

        // Find the largest contour in both frames
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

        // if there is no yellow regions on a side
        // that side should be a Skystone

        return input;
        // return the mat with rectangles drawn
    }

    public junctionLocation getLocation() {
        return this.location;
    }
}