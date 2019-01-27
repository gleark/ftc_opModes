
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Hardware_JEDC_v2
{
    // Public OpMode members.
    public DcMotor leftDrive   = null;
    public DcMotor rightDrive   = null;

    public Servo redFlag = null;
    public Servo blueFlag = null;

    public ColorSensor frontSensor = null;

    public BNO055IMU imu;

    // DECLARE opMode CONSTANTS HERE
    public double BLUE_FLAG_DOWN  = 0.84;
    public double BLUE_FLAG_UP    = 0.38;

    public double RED_FLAG_DOWN   = 0.29;
    public double RED_FLAG_UP     = 0.74;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    Telemetry telemetry;

    private ElapsedTime timer  = new ElapsedTime();

    /* Constructor */
    public Hardware_JEDC_v2() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "motorZero");
        rightDrive = hwMap.get(DcMotor.class, "motorOne");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Define and Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Servos
        blueFlag = hwMap.get(Servo.class, "blueServo");
        redFlag = hwMap.get(Servo.class,  "redServo");

        blueFlag.setPosition(BLUE_FLAG_DOWN);
        redFlag.setPosition(RED_FLAG_DOWN);

        frontSensor = hwMap.get(ColorSensor.class, "colorSensor");

    }

//    public void callibrateGyro() {
//        // start calibrating the gyro.
//        timer.reset();
//        while (timer.milliseconds() < 1000){
//            //wait
//            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
//            telemetry.update();
//        }
//
//        //Thread.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servo)
//
//        imu.initialize(true);
//
//
//        // make sure the gyro is calibrated.
//        while (imu.initialize())  {
//            //Thread.sleep(50);
//            telemetry.update();
//        }
//
//        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
//        telemetry.update();
//    }
}
