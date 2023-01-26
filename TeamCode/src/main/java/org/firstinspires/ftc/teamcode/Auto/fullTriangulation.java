package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teleop.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import java.util.ArrayList;
import java.util.List;
import org.opencv.imgproc.Moments;

@Autonomous (name="fullTriangulation")

public class fullTriangulation extends LinearOpMode{
    @Override
    public void runOpMode() {
    //public static void main(String[] args) {

        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Initialize two cameras
        VideoCapture camera1 = new VideoCapture(0);
        VideoCapture camera2 = new VideoCapture(1);


        // Intrinsic camera parameters

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

        // you should calibrate your cameras first and set the cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R, T, E, F, R1, R2, P1, P2, Q
        // ...


        //while (true) {
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
            telemetry.addData("3D Position of the object: ", objectPoints.toString());

        waitForStart();
        if (isStopRequested()) return;


/*
            // ...
            // Center the robot with the object by moving it towards the object
            // You can use the position data and the robot's motion control API
            // to move the robot towards the object until it is centered
            double x = objectPoints.get(0, 0)[0];
            double y = objectPoints.get(0, 0)[1];
            double z = objectPoints.get(0, 0)[2];

            // Move the robot towards the object
            // ...

            // Check if the robot is centered with the object
            double error = 0.1; // set a threshold for centering error
            if (Math.abs(x) < error && Math.abs(y) < error && Math.abs(z) < error) {
                System.out.println("Robot is centered with the object");

                // stop the robot
                // ...
                break;
            }

 */



       // }
    }
}