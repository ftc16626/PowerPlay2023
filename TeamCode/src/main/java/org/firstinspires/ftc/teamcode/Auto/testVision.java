package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "testVision")

public class testVision extends LinearOpMode {

    // position variables
    private int liftPos;
    private double bucketPos;
    private double distanceToHub;
    private double bucketDumpPos;

    //define dcmotors
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor arm = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    private OpenCvWebcam webcam;

    public Servo claw;

    //drivetrain variables (would have been used in drive method at the bottom)
    int leftPos;
    int rightPos;

    //arm variable
    int armPos;




    final int width = 320;
    final int height = 240;
    static final Point perfectPoint = new Point(160,120);

    final double perfectPointX = 160;
    final double perfectPointY = 120;

    boolean calculation = false;

    public double rightJunction = 0;
    public double rightDistance = 0;
    public double moveRobot = 0;



    triangulationTesting detector = new triangulationTesting(width);


    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.dcMotor.get("arm");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        //specifies that the devices are using encoders which allow for encoder (ticks)-based autonomous rather than timebased
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Because the motors are inverted (facing the center) on the robot, you have to reverse one side in order to make all the wheels move the same direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //these lines reset the encoders to 0 so that every time you use a command that uses encoders, you start from 0 rather than updating a value
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //one of the lift motors is reversed so that the lift can move up and down
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //this calls the sample mecanum drive from roadrunner (check the drive package to the left)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



//        initialize camera and pipeline and feed this opmode into cv
        //testAuto cv = new testAuto(this,detector);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

//      call the function to startStreaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.setPipeline(detector);
                //start streaming the camera
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //if you are using dashboard, update dashboard camera view
                /*FtcDashboard.getInstance().startCameraStream(webcam, 5);*/
                /*Point center = new Point();
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
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //cv.observeStick();

        waitForStart();

        calculation = true;

        if (isStopRequested()) {
            webcam.stopStreaming();
            return;
        }
        while(!isStopRequested()){
            rightJunction = 320 - detector.getCenterX();
            rightDistance = 320 - perfectPointX;
            moveRobot = detector.getCenterX() - perfectPointX;

            telemetry.addData("moveRobot", moveRobot);
            telemetry.addData("Center", detector.getCenterX());
            telemetry.addData("input matrix size", detector.getInputChecker().size());
            telemetry.addData("hierarchy matrix size", detector.getHierarchy().size());
            telemetry.addData("contours list size", detector.getContours().size());
            telemetry.addData("junction center viewport coordinates", detector.getCenter());
            for(int i=0;i<detector.getColorTester().cols();i++) {
                telemetry.addData("color tester element", detector.getColorTester().col(i).row(0));
            }
            telemetry.update();
        }

        sleep(1000000000);









        /*while (opModeIsActive()) {
        }
         */
//        stopStreaming
    }
}