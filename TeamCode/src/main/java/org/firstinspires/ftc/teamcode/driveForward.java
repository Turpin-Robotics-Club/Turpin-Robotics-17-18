package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.move;

@Autonomous(name="A C 0", group="Autonomous Finals")
//@Disabled
public class driveForward extends LinearOpMode{

    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    private Servo clamp;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;

    public void runOpMode()
    {


        new move(this);


        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        clamp = hardwareMap.servo.get("clamp");
        flmotor = hardwareMap.get(DcMotor.class, "front_left");
        frmotor = hardwareMap.get(DcMotor.class, "front_right");
        blmotor = hardwareMap.get(DcMotor.class, "back_left");
        brmotor = hardwareMap.get(DcMotor.class, "back_right");
        flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        clamp.setPosition(0.85);
        sleep(100);
        liftMotor.setPower(0.5);
        liftMotor2.setPower(0.5);
        sleep(100);
        liftMotor.setPower(0);
        liftMotor2.setPower(0);
        sleep(100);
        flmotor.setPower(0.75);
        frmotor.setPower(0.75);
        blmotor.setPower(0.75);
        brmotor.setPower(0.75);
        sleep(1000);
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);
        clamp.setPosition(0.7);
        sleep(100);
        liftMotor.setPower(0.5);
        liftMotor2.setPower(0.5);
        sleep(300);
        liftMotor.setPower(0);
        liftMotor2.setPower(0);
        flmotor.setPower(-0.5);
        frmotor.setPower(-0.5);
        blmotor.setPower(-0.5);
        brmotor.setPower(-0.5);
        sleep(100);

    }
}
