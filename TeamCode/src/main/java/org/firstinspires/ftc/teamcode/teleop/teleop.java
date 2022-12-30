package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Teleop")
public class teleop extends LinearOpMode {
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, lift1, lift2;
    public Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        backRight = hardwareMap.dcMotor.get("BackRight");
        arm = hardwareMap.dcMotor.get("arm");
        lift1 = hardwareMap.dcMotor.get("liftMotor1");
        lift2 = hardwareMap.dcMotor.get("liftMotor2");
        claw = hardwareMap.servo.get("claw");
    }
}
