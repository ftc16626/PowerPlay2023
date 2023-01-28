package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous (name="lowJunctionAuto")

public class strat2Auto extends LinearOpMode {

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

    public Servo claw;

    //drivetrain variables (would have been used in drive method at the bottom)
    int leftPos;
    int rightPos;

    //arm variable
    int armPos;


    @Override
    public void runOpMode() {
        //get motors from the hardware map (in the quotations are what the hardware objects are called in the configurations part of the driver station)
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



        /* The generation of the trajectories start here. You can name them whatever you want, but for
        this autonomous I made trajectories from 0 on. */


        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d(-39.0, -63.0, Math.toRadians(180.0)))
                .forward(5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(34)
                .build();


        //forward to cone
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(35)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(19)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(35)
                .build();

        /*Trajectory traj5 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(0)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    move(0.5, -1200, 700);
                    // Run your action in here!
                })
                .build();
         */


        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-70))))
                .forward(8)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(10)
                .build();


        // Red

        Trajectory traj10 = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(26)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(20)
                .build();


        // Green

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(14)
                .build();

        // Blue

        Trajectory traj13 = drive.trajectoryBuilder(traj11.end())
                .strafeRight(30)
                .build();

        Trajectory traj14 = drive.trajectoryBuilder(traj11.end())
                .back(14)
                .build();


        final int width = 320;
        final int height = 240;
        boolean test = false;


        // This has to do with the computer vision. The other file in this package called SignalReader is the camera configuration portion
        SignalReader detector = new SignalReader(width);
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        camera.setPipeline(detector);
        //camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
        // These statements below show up on the driver station and detect the average amounts of RGB in the frame created in SignalReader
        while (!isStarted()) {
            telemetry.addData("Avg Red in View", detector.getAvgRed());
            telemetry.addData("Avg Green in View", detector.getAvgGreen());
            telemetry.addData("Avg Blue in View", detector.getAvgBlue());
            telemetry.update();
        }


        //these set the position variables to 0 at the beginning
        int leftPos = 0;
        int rightPos = 0;

        int armPos = 0;

        int liftPos = 0;

        //closes the claw on the preloaded cone before the autonomous is initialized
        claw.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;
            //forward

            drive.followTrajectory(traj0);
            //lift arm and claw
            move(0.5, -150, 0);
            claw.setPosition(0);
            //strafe left
            drive.followTrajectory(traj1);
            //moves forward to low juction
            drive.followTrajectory(traj2);
            move(0.5, -200, 500);
            //turns towards cone stack
            drive.turn(Math.toRadians(-145));
            //should pick up cone and score from cone stack
            //make sure to tune values to cone height
            //ASSIGNMENT: I forgot about the preloaded cone when making this, so fix this so that it scores the preloaded cone at the beginning and picks up the cone for the high junction at the end
            claw.setPosition(1.2);
            sleep(1000);
            move(0.5, 200, 0);
            claw.setPosition(0);
            sleep(1000);
            move(0.5, -950, 0);
            sleep(1000);
            claw.setPosition(1.5);
            move(0.5, 950, 0);
            sleep(1000);

/*
            claw.setPosition(1.5);
            move(0.5, 200, 0);
            claw.setPosition(0);
            move(0.5, -950, 0);
            claw.setPosition(1.5);
            move(0.5, 950, 0);

            claw.setPosition(1.5);
            move(0.5, 200, 0);
            claw.setPosition(0);
            move(0.5, -950, 0);
            claw.setPosition(1.5);
            move(0.5, 950, 0);

            claw.setPosition(1.5);
            move(0.5, 200, 0);
            claw.setPosition(0);
            move(0.5, -950, 0);
            claw.setPosition(1.5);
            move(0.5, 950, 0);

            claw.setPosition(1.5);
            move(0.5, 200, 0);
            claw.setPosition(0);
            move(0.5, -950, 0);
            claw.setPosition(1.5);
            move(0.5, 950, 0);

            //should have scored all low junction cones. Now go to high junction

            drive.turn(Math.toRadians(-45));
            //moves forward
            drive.followTrajectory(traj3);
            /** ASSIGNMENT: go look at roadrunner and see if you can prevent the problem that arises here.
            The problem here is that when the robot moves forward and strafes right, there is a lot of wheel slippage. There are 2 solutions for this:
            Solution 1: the slippage gets accounted for because now the robot stops to score cones on the low junction and doesn't speed up as much. Test this pls:)
            Solution 2: You can use a splining method from roadrunner and make the robot strafe right without stopping. This would account for the wheel slippage that happens during de-acceleration
             */
            //strafes right
            //drive.followTrajectory(traj4);

            //Here is where you would call on the cameraCompute algorithm. It should get the contours of the high junction, calculate the centroid, and center the robot
            // for now, lets just do it manually
            /*drive.turn(Math.toRadians(-59));
            move(0.5, -1000, 800); // -1200 900 lift before and worked
            //moves forward towards pole
            drive.followTrajectory(traj5);
            move(0.5, 300, 800);
            claw.setPosition(0.3);
            // raises up arm above pole
            move(0.5, -200, 0);
            //move back
            drive.followTrajectory(traj6);
            //turns back so it is straight
            drive.turn(Math.toRadians(59)); //200
            //drops lift and arm
            move(0.5, -100, -900);

             */


            //fix parking after get the first part down
            //RED
            if (detector.getAvgRed() > detector.getAvgBlue() && detector.getAvgRed() > detector.getAvgGreen()) {
                drive.followTrajectory(traj10);
                drive.followTrajectory(traj11);


            }

            //GREEN
            if (detector.getAvgGreen() > detector.getAvgRed() && detector.getAvgGreen() > detector.getAvgBlue()) {
                drive.followTrajectory(traj12);

            }

            //BLUE

            if (detector.getAvgBlue() > detector.getAvgRed() && detector.getAvgBlue() > detector.getAvgGreen()) {
                drive.followTrajectory(traj13);
                drive.followTrajectory(traj14);
            }

        /** ASSIGNMENT:
         * After all of this works, make a second autonomous that is for the other side of the field. All you do is reverse most of the properties to make a reflection
         */

    }


        //private void drive(int leftTarget, int rightTarget, double speed, int armTarget, int liftTarget){
        // this is a custom method that updates the speed, the arm value, and the lift values during the autonomous (see above)
        private void move ( double speed, int armTarget, int liftTarget){

            //leftPos += leftTarget;
            //rightPos += rightTarget;
            armPos += armTarget;
            liftPos += liftTarget;

        /*leftFront.setTargetPosition(leftPos);
        leftBack.setTargetPosition(leftPos);
        rightFront.setTargetPosition(rightPos);
        rightBack.setTargetPosition(rightPos);
         */
            arm.setTargetPosition(armPos);
            liftMotor1.setTargetPosition(liftPos);
            liftMotor2.setTargetPosition(liftPos);




        /*leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //lift motors
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
         */
            arm.setPower(speed);
            liftMotor1.setPower(speed);
            liftMotor2.setPower(speed);
        }
    }


