package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;

public class move {

    private DcMotor flmotor;
    private DcMotor frmotor;
    private DcMotor blmotor;
    private DcMotor brmotor;

    private Servo ljewel;
    private Servo rjewel;
    private Servo relicServo;
    private Servo clamp;
    private Servo clamp2;

    private static Telemetry telemetry;
    TouchSensor reddish;
    LinearOpMode opMode;
    public boolean red;
    private double initGyroPos = 0;
    private double stabilityMultiplier = 0.0001;
    private double spinRate = 0.002;


    public static void pause(int milliseconds)
    {
        ElapsedTime tim = new ElapsedTime();
        tim.reset();
        while (tim.milliseconds()<milliseconds);
    }

    /**
     * Initializes motor variables
     * @param op The instance of the calling LinearOpMode
     *
     */
    public move(LinearOpMode op) {
        opMode = op;
        move.telemetry = op.telemetry;
        HardwareMap hardware_map = op.hardwareMap;
        reddish = hardware_map.touchSensor.get("touch");
        clamp = hardware_map.servo.get("clamp");
        clamp2 = hardware_map.servo.get("clamp2");
        ljewel = hardware_map.servo.get("raisin");
        rjewel = hardware_map.servo.get("raisin2");
        relicServo = hardware_map.servo.get("relic2");


        red = !reddish.isPressed();
        if (red) {
            flmotor = hardware_map.get(DcMotor.class, "front_left");
            frmotor = hardware_map.get(DcMotor.class, "front_right");
            blmotor = hardware_map.get(DcMotor.class, "back_left");
            brmotor = hardware_map.get(DcMotor.class, "back_right");
            flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            frmotor = hardware_map.get(DcMotor.class, "front_left");
            flmotor = hardware_map.get(DcMotor.class, "front_right");
            brmotor = hardware_map.get(DcMotor.class, "back_left");
            blmotor = hardware_map.get(DcMotor.class, "back_right");
            frmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            brmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }



        Sensors.initialize(opMode, red);
        resetEncoders();
        clamp.setPosition(0.95);
        clamp2.setPosition(1-0.95); //close
        relicServo.setPosition(0.09);
        pause(500);
        ljewel.setPosition(0.4);
    }


    public void resetEncoders()
    {
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void freeze()
    {
        int flpos = flmotor.getCurrentPosition();
        int frpos = frmotor.getCurrentPosition();
        int blpos = blmotor.getCurrentPosition();
        int brpos = brmotor.getCurrentPosition();

        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flmotor.setTargetPosition(flpos);
        frmotor.setTargetPosition(frpos);
        blmotor.setTargetPosition(blpos);
        brmotor.setTargetPosition(brpos);
        pause(1000);
        while(
                !(flpos+5 >flmotor.getCurrentPosition()&&flpos-5<flmotor.getCurrentPosition()) &&
                !(frpos+5 >frmotor.getCurrentPosition()&&frpos-5<frmotor.getCurrentPosition()) &&
                !(blpos+5 >blmotor.getCurrentPosition()&&blpos-5<blmotor.getCurrentPosition()) &&
                !(brpos+5 >brmotor.getCurrentPosition()&&brpos-5<brmotor.getCurrentPosition())
                );
        resetEncoders();
    }
    public void holdDirection()
    {
        /*
        initGyroPos = Sensors.readGyro();
        if (Sensors.readGyro() > initGyroPos) {
            flmotor.setPower(flmotor.getPower() + (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            blmotor.setPower(blmotor.getPower() + (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            frmotor.setPower(frmotor.getPower() - (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            brmotor.setPower(brmotor.getPower() - (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
        }
        if (Sensors.readGyro() < initGyroPos) {
            flmotor.setPower(flmotor.getPower() - (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            blmotor.setPower(blmotor.getPower() - (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            frmotor.setPower(frmotor.getPower() + (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
            brmotor.setPower(brmotor.getPower() + (Math.abs((oldSensors.gyro.rawZ() - initGyroPos)) * stabilityMultiplier));
        }
        telemetry.addData("Gyro Z", oldSensors.gyro.rawZ());
        telemetry.update();
        */
    }



    public void turnRight(double degrees, double power){
        double angleMod = 0.2;
        resetEncoders();
        double CIRCUMFERENCE = Math.PI * RobotConstants.wheelDiameter;
        double ROTATIONS = angleMod * degrees / CIRCUMFERENCE;
        double COUNTS = RobotConstants.encoderCPR * ROTATIONS * RobotConstants.gearRatio;

        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        if (degrees < 0) {

            flmotor.setPower(-power*RobotConstants.flpower);
            frmotor.setPower(power*RobotConstants.frpower);
            blmotor.setPower(-power*RobotConstants.blpower);
            brmotor.setPower(power*RobotConstants.brpower);

            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() > COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        } else {

            flmotor.setPower(power*RobotConstants.flpower);
            frmotor.setPower(-power*RobotConstants.frpower);
            blmotor.setPower(power*RobotConstants.blpower);
            brmotor.setPower(-power*RobotConstants.brpower);

            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() < COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        }

        resetEncoders();
        telemetry.addData("it has", "begun");
        telemetry.update();
        pause(1000);

    }

    /**
     * Moves the robot forward or backward
     *
     * @param distance Distance (in inches) for the robot to go. Positive for forward, negative for backward
     * @param power    The power level for the robot to move at. Should be an interval of [0.0, 1.0]
     * @throws InterruptedException
     */
    public void forward(double distance, double power){
        resetEncoders();
        double CIRCUMFERENCE = Math.PI * RobotConstants.wheelDiameter;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = RobotConstants.encoderCPR * ROTATIONS * RobotConstants.gearRatio;

        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flmotor.setTargetPosition((int) COUNTS);
        frmotor.setTargetPosition((int) COUNTS);
        blmotor.setTargetPosition((int) COUNTS);
        brmotor.setTargetPosition((int) COUNTS);


        flmotor.setPower(power*RobotConstants.flpower);
        frmotor.setPower(power*RobotConstants.frpower);
        blmotor.setPower(power*RobotConstants.blpower);
        brmotor.setPower(power*RobotConstants.brpower);

        if (distance < 0) {

            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() > COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() < COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        }

        resetEncoders();
        telemetry.addData("it has", "begun");
        telemetry.update();
        pause(1000);


    }




    /**
     * Move the robot right or negative right
     *
     * @param distance Distance (in inches) for the robot to move side to side. Positive for left, negative for right
     * @param power    The power level for the robot to move at. Should be an interval of [0.0, 1.0]
     */
    public void right(double distance, double power){

        resetEncoders();
        double CIRCUMFERENCE = Math.PI * RobotConstants.wheelDiameter;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = RobotConstants.encoderCPR * ROTATIONS * RobotConstants.gearRatio * RobotConstants.sidewaysModifier;

        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flmotor.setPower(power*RobotConstants.flpower);
        frmotor.setPower(-power*RobotConstants.frpower);
        blmotor.setPower(-power*RobotConstants.blpower);
        brmotor.setPower(power*RobotConstants.brpower);

        if (distance < 0) {

            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() > COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() < COUNTS) {
                telemetry.addData("front left counts", flmotor.getCurrentPosition());
                telemetry.addData("target", COUNTS);
                telemetry.update();
            }
        }

        resetEncoders();
        telemetry.addData("it has", "begun");
        telemetry.update();
        pause(1000);









    }

    /**
     * moves the robot to a column based on the red/blue value of move.java and the VuMark value from RobotConstants
     */
    public void toColumn()
    {
        RelicRecoveryVuMark relic = RobotConstants.vuMark;
        if(!red && relic == RelicRecoveryVuMark.LEFT) relic = RelicRecoveryVuMark.RIGHT;
        else if(!red && relic == RelicRecoveryVuMark.RIGHT) relic = RelicRecoveryVuMark.LEFT;
        if(relic == RelicRecoveryVuMark.CENTER);
        if(relic == RelicRecoveryVuMark.LEFT)
        {
            right(-10,-0.75);
        }
        if(relic == RelicRecoveryVuMark.RIGHT)
        {
            right(10,0.75);
        }

        forward(12, 0.75);
    }


    public void release()
    {
        clamp.setPosition(0.97);
        clamp2.setPosition(1-0.97);
    }

    public void lowerRaisin()
    {
        if(red)
        {
            rjewel.setPosition(0);
        }
        else{ljewel.setPosition(1);}
    }

    public void raisein()
    {
        if(red)
        {
            rjewel.setPosition(1);
        }
        else
        {
            ljewel.setPosition(0);
        }
    }


}
