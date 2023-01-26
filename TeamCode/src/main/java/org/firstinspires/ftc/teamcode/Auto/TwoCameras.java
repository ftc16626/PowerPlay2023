package org.firstinspires.ftc.teamcode.Auto;


import org.opencv.core.*;
import org.opencv.videoio.*;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.Scanner;

public class TwoCameras {
    private static Scanner scanner = new Scanner(System.in);

    public static void main(String[] args) {
        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Open two video capture objects
        VideoCapture camera1 = new VideoCapture(0);
        VideoCapture camera2 = new VideoCapture(1);

        // Check that the cameras were opened successfully
        if (!camera1.isOpened() || !camera2.isOpened()) {
            System.out.println("Error: Could not open cameras");

            return;
        }

        // Create two matrices to store the camera frames
        Mat frame1 = new Mat();
        Mat frame2 = new Mat();

        // Loop until a key is pressed
        while (true) {
            // Read a frame from each camera
            camera1.read(frame1);
            camera2.read(frame2);

            // Save the frames to image files
            Imgcodecs.imwrite("camera1.jpg", frame1);
            Imgcodecs.imwrite("camera2.jpg", frame2);

            // Check for a key press
            int key = waitKey(1);
            if (key != -1) {
                // A key was pressed, so break out of the loop
                break;
            }
        }

        // Release the cameras
        camera1.release();
        camera2.release();
    }

    public static int waitKey(int delay) {
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (scanner.hasNextLine()) {
            return (int) scanner.nextLine().charAt(0);
        }
        return -1;
    }
}