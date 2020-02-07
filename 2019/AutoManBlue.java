package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.text.DecimalFormat;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class AutoManBlue extends OpMode{

    // CONFIGURATION

    // Expansion Hub 1:
    // Motors:
    // Port 0: manArm
    // Servos
    // Port 0: leHand <-- Still needed
    // Sensors:
    // I2C Bus 0: Expansion Hub IMU
    // I2C Bus 1: frontDistance

    // Expansion Hub 2:
    // Motors:
    // Port 0: frontLeft
    // Port 1: rearLeft
    // Port 2: frontRight
    // Port 3: rearRight

    public DecimalFormat format = new DecimalFormat("##.00");
    private BNO055IMU imu;
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor manArm;

    private boolean haveBlock = false;

    private Servo elbow;
    private Servo hand;

    private DistanceSensor frontDistance;

    private String driveState = "driveForward"; // <-- Using string comparators to better explain...
                                        // ...what is happening with the autoState

    private String armState = "0block"; // <-- Using string comparators to better explain...
                                       // ...what is happening with the armState

    private int gyroCalibratedCount = 0;

    public Orientation angles;
    public Acceleration gravity;
    public double scalingFactor = 0.85;


    @Override
    public void init(){
        motorInit();
        gyroInit();
        sensorInit();
        hand.setPosition(0.3);
    }



    @Override
    public void init_loop(){
        gyroLoop();
    }

    @Override
    public void loop(){
        gyroLoop();
        measureDistance();
       driveLoop();
        moveArm();
        //setDrivePower();

    }

    private void motorInit(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        manArm = hardwareMap.get(DcMotor.class, "manArm");

        hand = hardwareMap.get(Servo.class, "leHand");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        manArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addData("MOTORS", "Initialized");
    }

    private void gyroInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private void sensorInit() {
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
    }

    private void gyroLoop() {
        //telemetry.addData("IMU", imu.isGyroCalibrated() ? "Initialized" : "Initializing...");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addData("ANGLE", angles);
        //telemetry.addData("HEADING", angles.firstAngle);
    }

    private void driveLoop(){

        switch (driveState){
            case "driveForward":
                if (!haveBlock) {
                  hand.setPosition(0.3);
                }
                frontLeft.setTargetPosition(-1376);
                frontRight.setTargetPosition(-1393);
                rearLeft.setTargetPosition(1376);
                rearRight.setTargetPosition(1347);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                rearLeft.setPower(0.3);
                rearRight.setPower(0.3);

                if (frontLeft.getCurrentPosition()<= -1360){
                    driveState = "backUpUno";
                }

            break;

            case "backUpUno":
                frontLeft.setTargetPosition(-505);
                frontRight.setTargetPosition(-513);
                rearLeft.setTargetPosition(476);
                rearRight.setTargetPosition(479);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                rearLeft.setPower(0.3);
                rearRight.setPower(0.3);

                if (frontLeft.getCurrentPosition()>= -500){
                    driveState = "chargeBackwards";
                }
            break;

            case "chargeBackwards":
                frontLeft.setTargetPosition(115);
                frontRight.setTargetPosition(-1259);
                rearLeft.setTargetPosition(-177);
                rearRight.setTargetPosition(1216);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(0.3);
                frontRight.setPower(0.3);
                rearLeft.setPower(0.3);
                rearRight.setPower(0.3);

                if (frontLeft.getCurrentPosition() >= 110 ){
                  driveState = "180oogabooga";
                }
            break;

            case "180oogabooga":
              // setDrivePower();
              frontLeft.setTargetPosition(2974);
              frontRight.setTargetPosition(1608);
              rearLeft.setTargetPosition(-3062);
              rearRight.setTargetPosition(-1609);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);

              if (frontLeft.getCurrentPosition() >= 2970){
                driveState = "turtle";
              }
            break;

            case "turtle":
              frontLeft.setTargetPosition(2255);
              frontRight.setTargetPosition(2334);
              rearLeft.setTargetPosition(-2307);
              rearRight.setTargetPosition(-2326);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);

              if (frontLeft.getCurrentPosition() <= 2260){
                driveState = "attackFoundation";
              }
            break;

            case "attackFoundation":
              frontLeft.setTargetPosition(2042);
              frontRight.setTargetPosition(2120);
              rearLeft.setTargetPosition(-2081);
              rearRight.setTargetPosition(-2115);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);

              if (frontLeft.getCurrentPosition() <= 2040){
                driveState = "dropBlock";
              }

          break;

          case "dropBlock":
            // moveClaw();
            hand.setPosition(0.3);
            setDrivePower();
            if (hand.getPosition()== 0.3){
              driveState = "kachowBackwards";
            }

          break;

          case "kachowBackwards":
              hand.setPosition(0.3);
              frontLeft.setTargetPosition(3040);
              frontRight.setTargetPosition(3097);
              rearLeft.setTargetPosition(-3106);
              rearRight.setTargetPosition(-3059);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);
              if (frontLeft.getCurrentPosition() >= 3030){
                driveState = "spinDoctor";
              }

            break;

            case "spinDoctor":
              armState = "0block";
              frontLeft.setTargetPosition(3612);
              frontRight.setTargetPosition(2510);
              rearLeft.setTargetPosition(-3723);
              rearRight.setTargetPosition(-2478);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);
              if (frontLeft.getCurrentPosition() >= 3610){
                driveState = "goUnderBridge";
              }
            break;

            case "goUnderBridge":
              // setDrivePower();
              frontLeft.setTargetPosition(1815);
              frontRight.setTargetPosition(736);
              rearLeft.setTargetPosition(-1939);
              rearRight.setTargetPosition(-738);

              frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              frontLeft.setPower(0.3);
              frontRight.setPower(0.3);
              rearLeft.setPower(0.3);
              rearRight.setPower(0.3);
              if (frontLeft.getCurrentPosition() <= 230){
                driveState = "weDoneBoi";
              }

            break;

            case "weDoneBoi":
              //setDrivePower();
              moveArm();
            break;


              // moveClaw();
            //   frontLeft.setTargetPosition(2500);
            //   frontRight.setTargetPosition(1250);
            //   rearLeft.setTargetPosition(-2500);
            //   rearRight.setTargetPosition(-200);

            //   frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //   frontLeft.setPower(-0.5);
            //   frontRight.setPower(0.5);
            //   rearLeft.setPower(0.5);
            //   rearRight.setPower(-0.5);

            // if (frontLeft.getCurrentPosition() >= 2600 ){
            //   armState = "0block";
            // }


        }
    }

    private void moveClaw() {
      haveBlock = false;
      hand.setPosition(0.3);
    }

    private void setDrivePower() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double rotation = -Math.pow(gamepad1.right_stick_x, 3.0) * -0.5;
        final double y = Math.pow(gamepad1.left_stick_y, 3.0) * -0.5;
        final double x = -Math.pow(gamepad1.left_stick_x, 3.0) * -0.5;

        final float currHeading = angles.firstAngle;
        final double direction = Math.atan2(x, y) + 0.0;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final float brake = 1.0f;

        final double lf = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rf = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;
        final double lr = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * brake * scalingFactor;
        final double rr = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * brake * scalingFactor;

        frontLeft.setPower(-lf);
        frontRight.setPower(-rf);
        rearLeft.setPower(lr);
        rearRight.setPower(rr);
    }

    private void moveArm() {
        //telemetry.addData("ManArm", manArm.getCurrentPosition());
        switch(armState){
          case "0block":
          manArm.setTargetPosition(0);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0 && !haveBlock) {
            haveBlock = true;
            hand.setPosition(1);
            if(hand.getPosition() == 1) {
              armState = "1block";
            }
          }
          break;

          case "1block":
          manArm.setTargetPosition(420);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;

          case "2block":
          manArm.setTargetPosition(600);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;

          case "3block":
          manArm.setTargetPosition(880);
          if(frontDistance.getDistance(DistanceUnit.CM) < 4.0) {
            hand.setPosition(1);
          }

          break;
        }
        manArm.setPower(0.3);
        manArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void measureDistance() {
        // telemetry.addData("frontDistance", String.format("%.01f cm", frontDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("BL", rearLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.addData("BR", rearRight.getCurrentPosition());
        telemetry.addData("driveState:", driveState);
  }


}
