package org.firstinspires.ftc.teamcode.Auto;


import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import java.util.ArrayList;
import java.util.List;
import org.opencv.imgproc.Moments;

public class YellowObjectDetector {
    public static void main(String[] args) {
        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Initialize two cameras
        VideoCapture camera1 = new VideoCapture(0);
        VideoCapture camera2 = new VideoCapture(1);

        while (true) {
            // Create matrices to store the frames from the cameras
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
            // Triangulate the position of the object by finding its centroid in both frames
            Moments moments1 = Imgproc.moments(largestContour1);
            Moments moments2 = Imgproc.moments(largestContour2);
            int x1 = (int)(moments1.get_m10() / moments1.get_m00());
            int y1 = (int)(moments1.get_m01() / moments1.get_m00());
            int x2 = (int)(moments2.get_m10() / moments2.get_m00());
            int y2 = (int)(moments2.get_m01() / moments2.get_m00());
            System.out.println("Position from camera 1: (" + x1 + ", " + y1 + ")");
            System.out.println("Position from camera 2: (" + x2 + ", " + y2 + ")");

            // You can use the position data from both cameras to triangulate the object's position in 3D space
            // or use other algorithms to calculate the position
            // ...
        }
    }
}