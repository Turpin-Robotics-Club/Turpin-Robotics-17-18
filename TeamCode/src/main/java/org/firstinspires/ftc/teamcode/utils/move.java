package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class move {

    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    static Telemetry telemetry;
    DcMotor spinLeft;
    DcMotor spinRight;
    Servo dump;
    LinearOpMode opMode;

    double initGyroPos = 0;
    double stabilityMultiplier = 0.0001;
    double spinRate = 0.002;

    int ENCODER_CPR = 1120;
    double GEAR_RATIO = 1;
    double WHEEL_DIAMETER = 5.94;

    /**
     * Initializes motor variables
     * @param op The instance of the calling LinearOpMode
     * @param red True if on the red Alliance, False otherwise
     */
    public move(LinearOpMode op, boolean red) {
        opMode = op;
        move.telemetry = op.telemetry;
        HardwareMap hardware_map = op.hardwareMap;

        dump = hardware_map.get(Servo.class, "servo_1");
        dump.setPosition(255);
        if (red) {
            flmotor = hardware_map.get(DcMotor.class, "motor_1");
            frmotor = hardware_map.get(DcMotor.class, "motor_2");
            blmotor = hardware_map.get(DcMotor.class, "motor_3");
            brmotor = hardware_map.get(DcMotor.class, "motor_4");
        } else {
            frmotor = hardware_map.get(DcMotor.class, "motor_1");
            flmotor = hardware_map.get(DcMotor.class, "motor_2");
            brmotor = hardware_map.get(DcMotor.class, "motor_3");
            blmotor = hardware_map.get(DcMotor.class, "motor_4");
        }
        frmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        oldSensors.initialize(opMode, red);
        resetEncoders();
    }


    public void resetEncoders()
    {
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void holdDirection()
    {
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
    }

    /**
     * Moves the robot forward or backward
     *
     * @param distance Distance (in inches) for the robot to go. Positive for forward, negative for backward
     * @param power    The power level for the robot to oldMove at. Should be an interval of [0.0, 1.0]
     * @throws InterruptedException
     */
    public void forward(double distance, double power){
        resetEncoders();
        double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flmotor.setTargetPosition((int) COUNTS);
        frmotor.setTargetPosition((int) COUNTS);
        blmotor.setTargetPosition((int) COUNTS);
        brmotor.setTargetPosition((int) COUNTS);


        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);

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

        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);

        resetEncoders();
        telemetry.addData("it has", "begun");
        telemetry.update();


    }

    public void dump()
    {
        dump.setPosition(0);
        while(opMode.opModeIsActive() && dump.getPosition() != 0);
        dump.setPosition(255);

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


    }

    /**
     * Move the robot left or right
     *
     * @param distance Distance (in inches) for the robot to oldMove side to side. Positive for left, negative for right
     * @param power    The power level for the robot to oldMove at. Should be an interval of [0.0, 1.0]
     * @throws InterruptedException
     */
    public void left(double distance, double power){
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

    }

    /**
     * Pivots the robot a certain degree around it's axis.
     *
     * @param degrees The amount (in degrees) to turn the robot. Positive for left, negative for right
     *
     */
    public void turnLeft(int degrees){
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





    }


    public void driveToLine(double power)
    {
        resetEncoders();
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive() && oldSensors.leye.green() < 3)
        {
            flmotor.setPower(power);
            frmotor.setPower(power);
            blmotor.setPower(power);
            brmotor.setPower(power);
            telemetry.addData("green", oldSensors.leye.green());
            telemetry.update();
        }

        resetEncoders();


    }


    public void powerUpShooter(DcMotor spin1, DcMotor spin2)
    {
        spinLeft = spin1;
        spinRight = spin2;
        spinRight.setDirection(DcMotor.Direction.REVERSE);

        spinLeft.setPower(-0.8);
        spinRight.setPower(-0.8);


    }



}
