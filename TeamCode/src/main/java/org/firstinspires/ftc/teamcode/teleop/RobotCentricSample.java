package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "RobotCentricSample")
public class RobotCentricSample extends LinearOpMode{

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor arm;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;
    public Servo claw;
    //public IMU imu;

    public final double liftPower = 1.0;
    public final double armPower = 1.0;
    public final int liftPickupPos = -350;
    public final int armPickupPosBack = -70;
    public final int armPickupPosFront = -3150;


    @Override
    public void runOpMode() throws InterruptedException {


        // Declare our motors
        // Make sure your ID's match your configuration
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        //imu = hardwareMap.IMU.get("imu");




        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        int liftPositionBottom = liftMotor1.getCurrentPosition();
        int liftPositionTop = liftPositionBottom - 1650;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx);
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx);
            double dtPowerFactor;

            if(gamepad1.left_bumper) dtPowerFactor = 0.3;
            else if(gamepad1.right_bumper) dtPowerFactor = -0.3;
            else dtPowerFactor = 1;

            leftFront.setPower(frontLeftPower * dtPowerFactor);
            leftBack.setPower(backLeftPower * dtPowerFactor);
            rightFront.setPower(frontRightPower * dtPowerFactor);
            rightBack.setPower(backRightPower * dtPowerFactor);

            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Lift Position", liftMotor1.getCurrentPosition());
            //IMU.getRobotYawPitchRollAngles();
            telemetry.update();


            /* Denominator is the largest motor power (absolute value) or 1
             This ensures all the powers maintain the same ratio, but only when
             at least one is out of the range [-1, 1]*/

            //Driver 2
            if (gamepad2.x){
                claw.setPosition(0); //close claw
            }
            else if (gamepad2.b){
                claw.setPosition(.15); //open claw
            }

            int liftCurrentPosition = liftMotor1.getCurrentPosition();
            int armCurrentPosition = arm.getCurrentPosition();

            //buttons for positioning the claw to pick up a cone
            if(gamepad2.y || gamepad2.a){
                //Open the claw.
                claw.setPosition(.15);

                //Set the arm to the proper position
                if(gamepad2.y)arm.setTargetPosition(armPickupPosFront);
                else arm.setTargetPosition(armPickupPosBack);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);

                //Make lift move to the target position.
                liftMotor1.setTargetPosition(liftPickupPos);
                liftMotor2.setTargetPosition(liftPickupPos);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad2.left_stick_y);

                liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //Lift-power factor needs to be less when the lift is moving downward because gravity also pulls the lift downward.
                double liftPowerFactor;
                if(gamepad2.right_stick_y>0) liftPowerFactor = 0.64;
                else liftPowerFactor = 0.8;

                telemetry.addData("gamepad 2 right stick y", gamepad2.right_stick_y);
                liftMotor1.setPower(gamepad2.right_stick_y * liftPowerFactor);
                liftMotor2.setPower(gamepad2.right_stick_y * liftPowerFactor);
            }



        }
    }
}