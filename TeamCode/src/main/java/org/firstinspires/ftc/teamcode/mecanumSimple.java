package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;


      
@TeleOp(name="Simple Drive")
public class mecanumSimple extends OpMode{

    private Servo clamp;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private boolean closed = false;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double liftPower;

    public final double SPEED = 0.75;
    public final double forwardBonus = 1.5;



    @Override
    public void init() {
        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        clamp = hardwareMap.servo.get("clamp");
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");


        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {



        // Movement
        frontLeftPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
        frontRightPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
        backLeftPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
        backRightPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));

        // Turning
        frontLeftPower += gamepad1.right_stick_x;
        frontRightPower += -gamepad1.right_stick_x;
        backLeftPower += gamepad1.right_stick_x;
        backRightPower += -gamepad1.right_stick_x;

        frontleft.setPower(frontLeftPower * SPEED);
        frontright.setPower(frontRightPower * SPEED);
        backleft.setPower(backLeftPower * SPEED);
        backright.setPower(backRightPower * SPEED);
       // Probably (should?) work- try to get lift motors mapped to right and left bumpers
        if (gamepad2.right_bumper)
        {
            liftMotor.setPower(0.75);
        }
        else if (gamepad2.right_trigger>0.5)
        {
            liftMotor.setPower(-gamepad2.right_trigger+0.5);
        }
        else
        {
            liftMotor.setPower(0);
        }
        //waiting for encoders
        //telemetry.addData("Lift", liftMotor.getCurrentPosition());

        if(gamepad2.a)
            clamp.setPosition(0.85); //close
        else if(gamepad2.b)
            clamp.setPosition(0.734); //open
        else if(gamepad2.y)
            clamp.setPosition(0.65); //extra open


    }

    @Override
    public void stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}
