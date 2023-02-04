package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class triangulationTesting extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    private int width;

    public triangulationTesting(int width) {
        frameList = new ArrayList<>();
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



    //Mat region = new Mat();
    public boolean maxContourT = false;

    public double centerX;
    public double centerY;
    Mat matSub = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();


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

        //region = firstFrame.submat(new Rect(region_pointA, region_pointB));
        matSub = firstFrame.submat(new Rect(region_pointA, region_pointB));
    }

    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        contours.clear();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        Point trial = new Point();
        trial.x = 200;
        trial.y = 200;
        //Imgproc.rectangle(input, region_pointA, region_pointB, BLUE, 2);
        Imgproc.circle(input, trial , 1, new Scalar(0, 0, 255), 2);

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
       // Core.inRange(mat, lowHSV, highHSV, thresh);
        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis


        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.

        /**MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
           // draw rectangle around boundaries
            Point center = new Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);
            centerX = boundRect[i].x + boundRect[i].width/2;
            centerY = boundRect[i].y + boundRect[i].height/2;
            Imgproc.circle(input, center, 4, new Scalar(0, 0, 255), 2);
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }
         **/

        //I don't exactly know what this was meant to do but it caused an error, so I commented it out - Aiden
//        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
//        Point center = rect.center;

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        /*double maxArea = 0;
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
            Imgproc.circle(input, center, 4, new Scalar(0, 0, 255), 2);

        }

         */


        // return the mat with rectangles drawn

        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        //region = input.submat(new Rect(region_pointA, region_pointB));

        return input;
    }


    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }

}
