package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.first
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous (name="Autonomous")
public class Auto extends LinearOpMode {
    private int liftPos;
    private double bucketPos;
    private double distanceToHub;
    private double bucketDumpPos;

    //define dcmotors
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor arm = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    public Servo claw; //Port?
    public ColorSensor colorSensor = null;

    //drivetrain variables
    int leftPos;
    int rightPos;

    //arm variables
    int armPos;



    @Override
    public void runOpMode() {
        final int width = 320;
        final int height = 240;
        boolean test = false;


        SignalReader detector = new SignalReader(width);
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
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
        while (!isStarted()) {
            telemetry.addData("Avg Red in View", detector.getAvgRed());
            telemetry.addData("Avg Green in View", detector.getAvgGreen());
            telemetry.addData("Avg Blue in View", detector.getAvgBlue());
            telemetry.update();
        }

        /*signal 1 = red
        signal 2 = green
        signal 3 = blue
        */
        boolean signal1 = detector.getAvgRed() > detector.getAvgBlue() + 10 && detector.getAvgRed() > detector.getAvgGreen() + 10;
        boolean signal3 = detector.getAvgBlue() > detector.getAvgRed() + 10 && detector.getAvgBlue() > detector.getAvgGreen() + 10;
        boolean signal2 = false;

        if(signal1 = false && signal3 = false){
            signal2 = true;
        }

        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        BackRight = hardwareMap.dcMotor.get("BackRight");

        claw = hardwareMap.servo.get("claw");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        arm = hardwareMap.dcMotor.get("arm");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // robot.FrontLeft.setTargetPosition(1000);
        //    robot.BackRight.setTargetPosition(-1000);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        int leftPos = 0;
        int rightPos = 0;

        int armPos = 0;

        int liftPos = 0;


        claw.setPosition(0);

        waitForStart();

        if(signal1){
            drive(-900, 900, 0.5, 0, 0);
            drive(800, 800, 0.5, 0, 0);
            drive(830, -830, 0.5, 0, 0);
            drive(1400, 1400, 0.5, 0, 0);
        }
        if(signal2){
            drive(500, 500, 0.5, 0, 0);

        }
        if(signal3){
            drive(400, 400, 0.5, 0, 0);
            drive(700, -700, 0.5, 0, 0);
            drive(1050, 1050, 0.5, 0, 0);
        }
        //lift.liftServo.setPosition(liftPos);
        //lift.bucketServo.setPosition(bucketPos);

        sleep(2000);

        if (isStopRequested()) return;


    }

    private void drive(int leftTarget, int rightTarget, double speed, int armTarget, int liftTarget){
        leftPos += leftTarget;
        rightPos += rightTarget;
        armPos += armTarget;
        liftPos += liftTarget;

        FrontLeft.setTargetPosition(leftPos);
        BackLeft.setTargetPosition(leftPos);
        FrontRight.setTargetPosition(rightPos);
        BackRight.setTargetPosition(rightPos);
        arm.setTargetPosition(armPos);
        liftMotor1.setTargetPosition(liftPos);
        liftMotor2.setTargetPosition(liftPos);




        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift motors
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
        arm.setPower(speed);
        liftMotor1.setPower(speed);
        liftMotor2.setPower(speed);
        sleep(1000);
}
