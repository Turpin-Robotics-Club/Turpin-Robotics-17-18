package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class move {

    private DcMotor flmotor;
    private DcMotor frmotor;
    private DcMotor blmotor;
    private DcMotor brmotor;

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
     *
     * @param distance the distance the robot should go
     * @param minPower the starting and ending speed
     * @param maxPower the maximum power the robot will run at
     * @param increment the speed at which the speed increases & decreases
     *
     */
    public void forward2(double distance, double minPower, double maxPower, double increment)
    {
        /*
        initGyroPos = oldSensors.gyro.rawZ();



        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        resetEncoders();
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("it has", "begun");
        telemetry.update();

        if(distance < 0)
        {
            minPower = -Math.abs(minPower);
            while(opMode.opModeIsActive() && (flmotor.getCurrentPosition()+frmotor.getCurrentPosition()+blmotor.getCurrentPosition()+brmotor.getCurrentPosition())/4 > COUNTS/1.85)
            {
                minPower = minPower - increment;
                flmotor.setPower(Math.max(minPower, maxPower));
                frmotor.setPower(Math.max(minPower, maxPower));
                blmotor.setPower(Math.max(minPower, maxPower));
                brmotor.setPower(Math.max(minPower, maxPower));
                holdDirection();
            }
            while(opMode.opModeIsActive() && (flmotor.getCurrentPosition()+frmotor.getCurrentPosition()+blmotor.getCurrentPosition()+brmotor.getCurrentPosition())/4 > COUNTS)
            {
                minPower = minPower + increment;
                flmotor.setPower(Math.max(minPower, maxPower));
                frmotor.setPower(Math.max(minPower, maxPower));
                blmotor.setPower(Math.max(minPower, maxPower));
                brmotor.setPower(Math.max(minPower, maxPower));
                holdDirection();
            }
        }
        else
        {
            minPower = Math.abs(minPower);
            while(opMode.opModeIsActive() && (flmotor.getCurrentPosition()+frmotor.getCurrentPosition()+blmotor.getCurrentPosition()+brmotor.getCurrentPosition())/4 < COUNTS/1.85)
            {
                minPower = minPower + increment;
                flmotor.setPower(Math.min(minPower, maxPower));
                frmotor.setPower(Math.min(minPower, maxPower));
                blmotor.setPower(Math.min(minPower, maxPower));
                brmotor.setPower(Math.min(minPower, maxPower));
                holdDirection();
            }
            while(opMode.opModeIsActive() && (flmotor.getCurrentPosition()+frmotor.getCurrentPosition()+blmotor.getCurrentPosition()+brmotor.getCurrentPosition())/4 < COUNTS)
            {
                minPower = minPower - increment;
                flmotor.setPower(Math.min(minPower, maxPower));
                frmotor.setPower(Math.min(minPower, maxPower));
                blmotor.setPower(Math.min(minPower, maxPower));
                brmotor.setPower(Math.min(minPower, maxPower));
                holdDirection();
            }
        }

        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);

        resetEncoders();

        */
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



        /*
        initGyroPos = oldSensors.gyro.getHeading();


        resetEncoders();
        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flmotor.setTargetPosition((int) -COUNTS);
        frmotor.setTargetPosition((int) COUNTS);
        blmotor.setTargetPosition((int) COUNTS);
        brmotor.setTargetPosition((int) -COUNTS);

        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);

        if (distance > 0) {

            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() > -COUNTS) {

                if (oldSensors.gyro.getHeading() - initGyroPos < 0) {
                    flmotor.setPower(flmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    blmotor.setPower(blmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    frmotor.setPower(frmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    brmotor.setPower(brmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                }
                if (oldSensors.gyro.getHeading() - initGyroPos > 0) {
                    flmotor.setPower(flmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    blmotor.setPower(blmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    frmotor.setPower(frmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    brmotor.setPower(brmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                }
            }
        } else {
            while (opMode.opModeIsActive() && flmotor.getCurrentPosition() < -COUNTS) {

                if (oldSensors.gyro.getHeading() - initGyroPos < 0) {
                    flmotor.setPower(flmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    blmotor.setPower(blmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    frmotor.setPower(frmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    brmotor.setPower(brmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                }
                if (oldSensors.gyro.getHeading() - initGyroPos > 0) {
                    flmotor.setPower(flmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    blmotor.setPower(blmotor.getPower() - (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    frmotor.setPower(frmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                    brmotor.setPower(brmotor.getPower() + (Math.pow((oldSensors.gyro.getHeading() - initGyroPos) * 2, 2) * stabilityMultiplier));
                }
            }
        }

        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);


        resetEncoders();
        */
    }

    /**
     * Pivots the robot a certain degree around it's axis.
     *
     * @param degrees The amount (in degrees) to turn the robot. Positive for left, negative for right
     *
     */
    public void turnLeft(int degrees){
        /*
        initGyroPos = Sensors.readGyro();
        resetEncoders();
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double target = initGyroPos - degrees;

        if (target < 0) {
            target = 360 + target;
        }
        if (target > 360) {
            target = target - 360;
        }
        while (opMode.opModeIsActive() && oldSensors.gyro.getHeading() != target) {
            telemetry.addData("Target:", target);
            telemetry.addData("Current:", oldSensors.gyro.getHeading());
            telemetry.addData("Delta:", target - oldSensors.gyro.getHeading());
            telemetry.update();

            flmotor.setPower(-(Math.pow(target - (oldSensors.gyro.getHeading() * 0.5), 2) * spinRate));
            blmotor.setPower(-(Math.pow(target - (oldSensors.gyro.getHeading() * 0.5), 2) * spinRate));
            frmotor.setPower((Math.pow(target - (oldSensors.gyro.getHeading() * 0.5), 2) * spinRate));
            brmotor.setPower((Math.pow(target - (oldSensors.gyro.getHeading() * 0.5), 2) * spinRate));
        }

        resetEncoders();
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);



        */

    }



}
