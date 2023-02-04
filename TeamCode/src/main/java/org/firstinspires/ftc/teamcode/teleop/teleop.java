package org.firstinspires.ftc.teamcode.teleop;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp (name="Teleop")
public class teleop extends LinearOpMode {
    public DcMotor leftFront, rightFront, leftBack, rightBack, arm, liftMotor1, liftMotor2;
    public Servo claw;

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        arm = hardwareMap.dcMotor.get("arm");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");
        claw = hardwareMap.servo.get("claw");
        boolean notAccessed = false;

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // robot.FrontLeft.setTargetPosition(1000);
        // robot.BackRight.setTargetPosition(-1000);

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
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        String logFilePath = String.format("%s/FIRST/data/myLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());

        try {
            File logFile = new File(logFilePath);
            FileWriter writer = new FileWriter(logFile);
            writer.write("word");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
            notAccessed=true;
        }

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Log File", logFilePath);
            if(notAccessed)
                telemetry.addLine("failure");
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}
