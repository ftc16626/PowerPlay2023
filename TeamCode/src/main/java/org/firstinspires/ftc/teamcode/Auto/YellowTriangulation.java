package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class YellowTriangulation {

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Mat image = Imgcodecs.imread("image.jpg");

        // Convert the image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define a range of yellow colors in HSV
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Threshold the image to only select yellow colors
        Mat yellowMask = new Mat();
        Core.inRange(hsvImage, lowerYellow, upperYellow, yellowMask);

        // Find the contours of the yellow object
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // Triangulate the position of the object by finding its centroid
        Moments moments = Imgproc.moments(largestContour);
        int x = (int)(moments.get_m10() / moments.get_m00());
        int y = (int)(moments.get_m01() / moments.get_m00());
        System.out.println("Position: (" + x + ", " + y + ")");
    }
}