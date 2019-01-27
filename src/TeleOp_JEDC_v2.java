package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tank Drive", group="TeleOp")
@Disabled


public class TeleOp_JEDC_v2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Hardware_JEDC_v2 robot = new Hardware_JEDC_v2();

    boolean lastBlueButton = false;
    boolean lastRedButton  = false;

    double redFlagPos  = robot.RED_FLAG_DOWN;
    double blueFlagPos = robot.BLUE_FLAG_DOWN;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.redFlag.setPosition(robot.RED_FLAG_DOWN);
        robot.blueFlag.setPosition(robot.BLUE_FLAG_DOWN);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        robot.imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating imu...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }

    }

    // Code to run REPEATEDLY after INIT, prior to START
    @Override
    public void init_loop() {

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        double left;
        double right;

        double csBlue, csRed, csGreen;

        csRed   = robot.frontSensor.red();
        csGreen = robot.frontSensor.green();
        csBlue  = robot.frontSensor.blue();

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(left);
        robot.leftDrive.setPower(right);

        if (gamepad1.x && !lastBlueButton) {
            blueFlagPos = robot.BLUE_FLAG_UP;
            lastBlueButton = true;
        } else {
            blueFlagPos = robot.BLUE_FLAG_DOWN;
            lastBlueButton = false;
        }

        if (gamepad1.b && !lastRedButton) {
            redFlagPos = robot.RED_FLAG_UP;
            lastRedButton = true;
        } else {
            redFlagPos = robot.RED_FLAG_DOWN;
            lastRedButton = false;
        }

        robot.redFlag.setPosition(redFlagPos);
        robot.blueFlag.setPosition(blueFlagPos);


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("bServo",  "%.2f", blueFlagPos);
        telemetry.addData("rServo", "%.2f", redFlagPos);
        telemetry.addData("R", csRed);
        telemetry.addData("G", csGreen);
        telemetry.addData("B", csBlue);

        telemetry.update();

    }


    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.redFlag.setPosition(0.1);
        robot.blueFlag.setPosition(0.1);

    }





}
