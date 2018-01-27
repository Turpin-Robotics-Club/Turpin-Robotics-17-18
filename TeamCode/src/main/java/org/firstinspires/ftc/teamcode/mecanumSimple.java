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
    private Servo clamp2;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor liftMotor3;
    private DcMotor relic;
    private boolean closed = false;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private Servo relicServo;
    private Servo ljewel;
    private Servo rjewel;



    private final double SPEED = 0.75;
    private final double forwardBonus = 1.5;



    @Override
    public void init() {
        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor2 = hardwareMap.dcMotor.get("lift2");
        liftMotor3 = hardwareMap.dcMotor.get("lift3");
        clamp = hardwareMap.servo.get("clamp");
        clamp2 = hardwareMap.servo.get("clamp2");
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");
        relic = hardwareMap.dcMotor.get("relic");
        relicServo = hardwareMap.servo.get("relic2");
        ljewel = hardwareMap.servo.get("raisin");
        rjewel = hardwareMap.servo.get("raisin2");



        relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //relic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {



        // Movement
        if(gamepad1.right_bumper)
        {
            frontLeftPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus))/3;
            frontRightPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus))/3;
            backLeftPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus))/3;
            backRightPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus))/3;
        }


        else {
            frontLeftPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
            frontRightPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
            backLeftPower = (-gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
            backRightPower = (gamepad1.left_stick_x + (-gamepad1.left_stick_y * forwardBonus));
        }



        // Turning
        frontLeftPower += gamepad1.right_stick_x;
        frontRightPower += -gamepad1.right_stick_x;
        backLeftPower += gamepad1.right_stick_x;
        backRightPower += -gamepad1.right_stick_x;

        frontleft.setPower(frontLeftPower * SPEED);
        frontright.setPower(frontRightPower * SPEED);
        backleft.setPower(backLeftPower * SPEED);
        backright.setPower(backRightPower * SPEED);


        telemetry.addData("Lift3", liftMotor3.getCurrentPosition());
       // Probably (should?) work- try to get lift motors mapped to right and left bumpers
        if (gamepad2.right_bumper)
        {
            liftMotor.setPower(0.6);
            liftMotor2.setPower(0.6);
            liftMotor3.setPower(0.6*0.3);
        }
        else if (gamepad2.right_trigger>0.6)
        {
            liftMotor.setPower(-(gamepad2.right_trigger/2)+0.35);
            liftMotor2.setPower(-(gamepad2.right_trigger/2)+0.35);
            liftMotor3.setPower((-(gamepad2.right_trigger/2)+0.25)*1.2);
        }
        else if(gamepad2.left_bumper)
        {
            liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(0);
            liftMotor2.setTargetPosition(0);
            liftMotor3.setTargetPosition(0);
            liftMotor.setPower(0.8);
            liftMotor2.setPower(0.8);
            liftMotor3.setPower(0.8);
            telemetry.addData("Lift", liftMotor.getCurrentPosition());
            telemetry.addData("Lift2", liftMotor2.getCurrentPosition());



        }
        else
        {
            liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
            liftMotor3.setPower(0);
        }
        //waiting for encoders
        telemetry.addData("Lift", liftMotor.getCurrentPosition());

        if(gamepad2.a){
            clamp.setPosition(0.95);
            clamp2.setPosition(1-0.95);} //close
        else if(gamepad2.b){
            clamp.setPosition(0.962);
            clamp2.setPosition(1-0.962);}//open
        else if(gamepad2.y) {
            clamp.setPosition(0.99);
            clamp2.setPosition(1-0.99);}//extra open



        if(gamepad2.dpad_right) relic.setPower(1);
        else if(gamepad2.dpad_left) relic.setPower(-1);
        else relic.setPower(0);
        telemetry.addData("Relic Position",relic.getCurrentPosition());

        if (gamepad2.dpad_up) relicServo.setPosition(0.08); //up
        else if(gamepad2.dpad_down) relicServo.setPosition(0.025); //down

        if(gamepad1.x) {ljewel.setPosition(1);rjewel.setPosition(0);}
        if(gamepad1.y) {ljewel.setPosition(0);rjewel.setPosition(1);}

        if(gamepad2.right_stick_y>0.05||gamepad2.right_stick_y<-0.05) relicServo.setPosition(relicServo.getPosition()-(gamepad2.right_stick_y*0.0005));


    }

    @Override
    public void stop() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}
