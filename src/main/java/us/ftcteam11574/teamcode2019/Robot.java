package us.ftcteam11574.teamcode2019;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class Robot {
    public static ElapsedTime runtime = new ElapsedTime();
    public static DcMotor tl = null; //top left
    public static DcMotor tr = null; //top right
    public static DcMotor bl = null; //bottom left
    public static DcMotor br = null; //bottom right
    public static DcMotor intakeR = null;
    public static DcMotor intakeL = null;
    public static DcMotor pantagraph = null;
    public static BNO055IMU imu;
    public static DcMotor[] motors; //all motors
    //public static ColorSensor mColorSensor; //too much lag for now
    public static Rev2mDistanceSensor mDistanceSensor;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static CRServo foundation1;
    public static CRServo foundation2;
    public static CRServo capstone;
    public static Servo piston1;
    public static Servo piston2;
    public static double power_foundation = 1; //start at the max power
    // /* CAMERA VV
    public static WebcamName webCam;
    public static VuforiaLocalizer vuforia;
    public static final float home = .4f;
    public static final float open = .55f;
    public static final float close = .3f;
    public static final float eps = .01f;
    // /* Camera ^^





    public static void init_teleOp(Telemetry telemetry_, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        overallInit(telemetry_,hardwareMap_,gamepad1_,gamepad2_);
        //NO webCam
        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    public static void init_autoOp(Telemetry telemetry_, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        overallInit(telemetry_,hardwareMap_,gamepad1_,gamepad2_);
        initCamera();
        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int i = 0; motors.length > i; i++) {
            motors[i].setTargetPosition(0);
            Robot.motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders to start
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    public static void overallInit(Telemetry telemetry_, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        hardwareMap= hardwareMap_;
        gamepad1=gamepad1_;
        gamepad2=gamepad2_;
        telemetry = telemetry_;
        telemetry.addData("Status", "Initialized");
        tl  = hardwareMap.get(DcMotor.class, "tl");  //45 degrees //v0
        tr  = hardwareMap.get(DcMotor.class, "tr");  //135 degrees //v1
        bl  = hardwareMap.get(DcMotor.class, "bl");  //225 degrees //v2
        br  = hardwareMap.get(DcMotor.class, "br");  //315 degrees //v3
        intakeR  = hardwareMap.get(DcMotor.class, "intakeR");  //225 degrees //v2
        intakeL  = hardwareMap.get(DcMotor.class, "intakeL");  //315 degrees //v3
        pantagraph  = hardwareMap.get(DcMotor.class, "pant");  //315 degrees //v3
        foundation1 = hardwareMap.get(CRServo.class,"foundation1");
        foundation2 = hardwareMap.get(CRServo.class,"foundation2");
        capstone = hardwareMap.get(CRServo.class,"capstone");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        gamepad1.setJoystickDeadzone(.02f);
        tl.setDirection(DcMotor.Direction.FORWARD);
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setDirection(DcMotor.Direction.FORWARD);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setDirection(DcMotor.Direction.FORWARD);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foundation1.setDirection(CRServo.Direction.REVERSE);
        foundation2.setDirection(CRServo.Direction.FORWARD);
        motors = new DcMotor[]{br,tr,tl,bl}; //
        mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        piston1 = hardwareMap.get(Servo.class,"Piston1");
        piston2 = hardwareMap.get(Servo.class,"Piston2");

         //This one is the correct one for now, I believe
        // /*


        //*/
        //mColorSensor = hardwareMap.colorSensor.get("color"); //I think its causing too much lag

        /*  ***CAMERA***
         //Wait until we get it to find the camera before trying to get this to work
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        vu_parameters.vuforiaLicenseKey = "AV29AFb/////AAABma3tuKm8DE2/tKJA0LIvwcIWOzMsiVbx8yLAiSRl1l98p84lwbzzJMkqsJw7ySFusaR6sYtQoSN9rzPIjUVqJ/uLkqv/V0rllY9LtZS0bnUfiyYarG+ZIDk587QhB/+BdT2EMo7w7+dHPO3Y9YOoFMZom016W6kYU+Tc7/OaN0AMXb6zGal02KRH3h913F+84o7J48sKXz0whgL1TSbfFQvYYyzijQlqzsmcvee4e3AI3L30L9AM1+COMhKcsIuYjpuUl1/oELl6XSCC7Q3UVnrKnah1WQb2C8m1KdsGgPbPp42rFC4ArXydJI193CEEENY/fyHvxxh8/aEb4fxxmybXkPk93BVpPZL6co8hFpSF";


        vu_parameters.cameraName = webCam;


        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);

        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        */




    }
    public static void initCamera() {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vu_parameters.vuforiaLicenseKey = "AV29AFb/////AAABma3tuKm8DE2/tKJA0LIvwcIWOzMsiVbx8yLAiSRl1l98p84lwbzzJMkqsJw7ySFusaR6sYtQoSN9rzPIjUVqJ/uLkqv/V0rllY9LtZS0bnUfiyYarG+ZIDk587QhB/+BdT2EMo7w7+dHPO3Y9YOoFMZom016W6kYU+Tc7/OaN0AMXb6zGal02KRH3h913F+84o7J48sKXz0whgL1TSbfFQvYYyzijQlqzsmcvee4e3AI3L30L9AM1+COMhKcsIuYjpuUl1/oELl6XSCC7Q3UVnrKnah1WQb2C8m1KdsGgPbPp42rFC4ArXydJI193CEEENY/fyHvxxh8/aEb4fxxmybXkPk93BVpPZL6co8hFpSF";
        vu_parameters.cameraName = webCam;
        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);
        vuforia.enableConvertFrameToBitmap();
        // vuforia.getCameraCalibration();
        //rgb_format_worked = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB888)[0]; //always fails
        vuforia.setFrameQueueCapacity(3);
    }
    public static double distanceTurn(double cur, double goalAngle) {
        //The way to calculate this is as follows:
        //if goalAngle is less than 0, than add 360
        //so angles from (-180,0] get turned into (180,360]
        //angles from (0,180] stay as (0,180]
        //then we can just find which is a shorter distance

        //then we need to convert the current angle to the zero to 360 thing
        //now compare the distance
        double ncur = convertAng(cur);         //90
        double ngoal = convertAng(goalAngle);  //270
        double dif = (ngoal-ncur);              //179 , 181
        //we have a problem when going straight backwards
        //it treats it the same as going straight forwards
        //and it thinks were much closer than we are it seems
        //lets consider the case of the cur=90 goal = -90 -> 270
        //dif = 180
        //dif = 360-180 = 180 (good)
        if (dif >= Math.PI) {
            return -((Math.PI*2)-dif);  //need to decrease angle
            //
        }
        else if (dif <= -Math.PI) {
            return(( Math.PI*2)+dif); //increase angle
        }
        else if(dif > 0 && dif < Math.PI) {
            return dif;
        }
        else if(dif < 0 && dif > -Math.PI) {
            return dif;
        }
        else {
            return 0;
        }



    }
    public static double convertAng(double ang) {
        //in the form [-180 to 180]
        if(ang < 0) {
            ang += Math.PI*2;
        }
        return ang;
    }
    public static double[] orientMode(double vx ,double vy ,double rot,double rot2) {


        rot2 = -rot2;
        //maybe need to swap rot2 and rot1
        //somethign else is a problem though,s
        double ang = Math.atan2(vy,-vx);
        double mag = Math.sqrt(vy*vy + vx*vx);
        //atan works like this
        //         90
        //        ^
        // +-180  <   > 0
        //      -90 V
        //having some trouble with resetting
        //When both 0, shoudl be equal to -90
        //At 0, as if 90, so goes forward (CHECK)
        //At 90, as if
        double imu_ang =(Robot.imu.getAngularOrientation().firstAngle + 0*Math.PI/2.0) % Math.PI;
        double goal_ang = (ang + (-imu_ang))  ; //This needs to be offset by some number of degrees, because when both are zero, it should move foward
        double ang_rot = Math.atan2(rot,rot2)- Math.PI/2.0; //angle your game stick is facing
        double mag_rot = Math.sqrt(rot*rot + rot2*rot2);
        double goal_rot_ang = distanceTurn(-imu_ang,ang_rot); //
        double nrot = 0;
        double nvx = Math.cos(goal_ang) * mag;
        double nvy = Math.sin(goal_ang) * mag;
        //only should be if max mode

        /*
        if(Math.abs(goal_rot_ang) > 0.40) { //amount goal angle needs to be off to actually turn in that direction

            nrot = goal_rot_ang * (mag_rot/2.0)  ; //rotation is the direction of rotation
            //test value of /2.0 just ot
            //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
        }
        else {

            nrot = goal_rot_ang * (mag_rot)*(Math.abs(goal_rot_ang)+.1)  ; //rotation is the direction of rotation
            // nrot = nrot>0?.1 + nrot:-.1+nrot;
        }

         */
        nrot = goal_rot_ang; //should work, hopefully, test in opNoDist thing

        double[] powers = motorPower.calcMotorsFull(nvx, nvy, nrot);//can also calc max, which always goes the fastest
        return powers;
    }
    public static void pantagraphDown(ElapsedTime time) {
        if (time.milliseconds() < 1500) {
            Robot.pantagraph.setPower(-1);
        }
        else {
            Robot.pantagraph.setPower(0);
        }
    }
    public static void pantagraphUp(ElapsedTime time) {
        if (time.milliseconds() < 1500) {
            Robot.pantagraph.setPower(1);
        }
        else {
            Robot.pantagraph.setPower(0);
        }
    }
    public static void reset_pow() {
        Robot.pantagraph.setPower(0);
        Robot.offtake();
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.motors[i].setPower(0); //this should reset all the motors
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Robot.foundation1.setPower(0);
        Robot.foundation2.setPower(0);
    }
    public static void intake() {
        Robot.intakeR.setPower(1);
        Robot.intakeL.setPower(1);
    }
    public static void intake(double pow) {
        Robot.intakeR.setPower(pow);
        Robot.intakeL.setPower(pow);
    }
    public static void outtake() {
        Robot.intakeR.setPower(-1);
        Robot.intakeL.setPower(-1);
    }
    public static void outtake(double pow) {
        Robot.intakeR.setPower(-pow);
        Robot.intakeL.setPower(-pow);
    }
    public static void offtake() {
        Robot.intakeR.setPower(0);
        Robot.intakeL.setPower(0);
    }
    /*
    public static int blockPosition(int[] center, int wide,int high) {
        final int low_hi = high + high/10;
        final int hi_hi = (int) (high/1.3);

        //final int low_right = 0;
        //final int hi_right = wide/3 - wide/10; //all of these values shoudl be experimentally found



        if(! (center[1] > low_hi && center[1] < hi_hi) ) {
            return -1; //0 if not found in reasonable spot
        }
        for (int i = 0; 3 > i; i++) {
            if(center[0] > i*(wide/3) && center[0] < (i+1)*wide/3) {
                return i;
            }
        }
        return -1;


    }
    */
    //returns where the cetner is
    public static int[] center() {
        Object[] res = (readImageRGB565(readCamera())); //in the form {rgb,width,height}
        int[] center = Skyblock.returnCenter( ((int[][]) (res[0])),(int)res[1],(int)res[2]);
        return center;
        //return blockPosition(center,(int)res[1],(int)res[2]);
    }

    public static Object[] readImageRGB565(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        java.nio.ByteBuffer pixels = image.getPixels();
        //System.out.println("the byte order is" + pixels.order());
        int[][] rgb = new int[width*height][3];
        //telemetry.addData("type",pixels.order());
        //ByteOrder.
        //ASSUMING THAT BYTEORDER IS BIG ENDIAN
        for(int i=0; pixels.hasRemaining(); i++) {
            int read1 = ((Byte)pixels.get()).intValue() << 8 | ((Byte)pixels.get()).intValue();
            int red_read = ( (read1 & 0b11111000_00000000) >> 11) << 3 ; //read the first 5 bits
            int green_read = ( (read1 & 0b00000111_11100000) >> 5) << 2;
            int blue_read = ( (read1 & 0b00000000_00011111) >> 0 ) << 3;
            rgb[i][0] =   red_read;
            rgb[i][1] =   green_read;
            rgb[i][2] =   blue_read;
        }
        return new Object[]{rgb,width,height};
    }

    public static Object[] readImageRGB888(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        java.nio.ByteBuffer pixels = image.getPixels();
        int i = 0;
        System.out.println("the byte order is" + pixels.order());
        int[][] rgb = new int[width*height][3];
        while(pixels.hasRemaining()) {
            int red = ((Byte)pixels.get()).intValue();
            int green = ((Byte)pixels.get()).intValue();
            int blue = ((Byte)pixels.get()).intValue();
            rgb[i][0]   = red;
            rgb[i][1]   = green;
            rgb[i++][2] = blue;
            //hopefully, this convert correctly, this will need some testing

            /*
            convertByte(red,pixels.order());
            */

            //WE ARE ASSUMING LENGTH OF PIXELS % 3 == 0 !!!
            //MIGHT CRASH!
            //
        }
        return new Object[]{rgb,width,height};

        //pixels are in the form:
        //RRRRRRRR GGGGGGGG BBBBBBB
    }
    public static Image readCamera() {
        VuforiaLocalizer.CloseableFrame frame = null;
        Image rgb = null;
        try {
            frame = vuforia.getFrameQueue().take();
            telemetry.addData("images",frame.getNumImages());
            long numImages = frame.getNumImages();
            for (int i = 0; numImages > i; i++) {
                telemetry.addData("rgb format",frame.getImage(i).getFormat());

                //right now only reading the color greyscale

                if(frame.getImage(i).getFormat()==PIXEL_FORMAT.RGB888) {

                    //rgb = frame.getImage(i);
                    //skip this for now
                }
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return rgb;
    }
    //now need some methods for travel distance
    public static void turnDistance(int distance, double speed) {
        //will need to figure out how the power works, shoudl it be negative, when distance is negative?, or is just based on the aboslute value
        //distance should be converted before calling this method
        for (int i = 0; motors.length > i; i++) {
            motors[i].setPower(speed);
            motors[i].setTargetPosition(distance);
        }
    }
    //ALSO
        //COULD CONSIDER PLAYING NOISE WHEN IT FINISHES
        //OR WHEN IT FINDS THE BLOCK
    //move based on direction

    public static void mmToDist(double mm) {
        //convert mm to Distance on the motors
        //TODO

    }
    public static int motorsFinished() {
        int finished = 0;
        for (int i =0; motors.length > i; i++) {
            //(motors[i].getCurrentPosition() > motors[i].getTargetPosition()) //need to check if its positve or negative first
            //might have problem with isBusy
            //!motors[i].isBusy() ||
            if ((motors[i].getPower() < 0 && (motors[i].getCurrentPosition() <= motors[i].getTargetPosition())) || (motors[i].getPower() > 0 && (motors[i].getCurrentPosition() >= motors[i].getTargetPosition()))) {
                finished++;
            }
        }
        return finished;
    }
    public static void resetTime() {
        runtime.reset();
    }

    public static double timeElapsed() {
        return runtime.milliseconds();
    }



    public static void moveDir(double dir, double speed,  int distance, boolean max) { //not rot aspect for now, since that might be super confusing
        Robot.moveVx(Math.cos(dir)*speed,Math.sin(dir)*speed,distance,max);

    }
    public static void moveVx(double vx, double vy, int distance, boolean max) { //not rot aspect for now, since that might be super confusing

        //distance should be converted before calling this method
       setMotors(vx,vy,0,max);
       for (int i = 0; motors.length > i; i++) {
           //not sure if this shoudl have Math.abs()??
           motors[i].setTargetPosition( (int) Math.abs((distance )) ); //we want to scale based on power, since the ones with less power should travel
           //equally less distance
       }
    }
    public static void moveVx(double vx, double vy, double rot, int distance, boolean max) { //not rot aspect for now, since that might be super confusing

        //distance should be converted before calling this method
        setMotors(vx,vy,rot,max);
        for (int i = 0; motors.length > i; i++) {
            //not sure if this shoudl have Math.abs()??
            motors[i].setTargetPosition( (int) Math.abs((distance )) ); //we want to scale based on power, since the ones with less power should travel
            //equally less distance
        }
    }



    public static void setMotors( double vx, double vy, double rot,boolean max_mode) {
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        if (max_mode) { //max speed always,maximym possible speed
            setMotorsMax(powers, 1);
        } else {
            //1.6, constant near where this reaches its max, (on the lower side, to ensure it doens't go over)
            for (int i = 0; powers.length > i; i++) {
                powers[i] *= 1.6;
            }
            setMotors(powers, max_abs(new double[]{Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2)), rot}));
        }
    }

    public static void setMotors(double[] powers, double len) {
        //powers motors with given ratios, and rescales so the max length is len.
        double max = max_abs(powers);
        double len_mult = 1;
        //does't chnage the motor speed unless it has to
        if (max > 1) {
            len_mult = len / max;
        }
        //telemetry.addData("MAX", "max " + max);
        //telemetry.addData("Power", "len_mult " + len_mult);
        for (int i = 0; powers.length > i; i++) {
            // telemetry.addData("Power motors",  powers[i] * len_mult);
            motors[i].setPower(powers[i] * len_mult);
        }

    }
    public static void setMotorsMax(double[] powers, double len) {
        //powers motors with given ratios, and rescales so the max length is len.
        double max = max_abs(powers);
        double len_mult = 1;


        if (Math.abs(max) < 0.01) {
            len_mult = 0;
        }
        else {
            len_mult = len / max;
        }

        //telemetry.addData("MAX", "max " + max);
        //telemetry.addData("Power", "len_mult " + len_mult);
        for (int i = 0; powers.length > i; i++) {
            motors[i].setPower(powers[i] * len_mult);
        }

    }
    public static double max_abs(double[] list) {
        double max = Math.abs(list[0]);
        //int id = 0;
        for (int i = 1; list.length > i; i++) {
            if (Math.abs(list[i]) > max) {
                max = Math.abs(list[i]);
                //id = i;
            }
        }
        return max;

    }
    public static boolean isBlock() {
        //lagging it out and not workign great for now

        /*
        final int[] block = new int[]{255,255,0};
        int[] color_read = new int[]{mColorSensor.red(), mColorSensor.green(), mColorSensor.blue()};
        //now meansure the distance between the colors, weight blue most highly, since yellow shouldn't have nearly any blue
        int dist = Math.abs(block[0]-color_read[0]) + Math.abs(block[1]-color_read[1]) + (Math.abs(block[2]-color_read[2])*5);

        //blue is weighted most strongly, 5 times as important for now
        //maybe see hwo alpha values get returned, and also put those in
        */
        //dist < 70 ||
        if (  mDistanceSensor.getDistance(DistanceUnit.MM) < 60) { //increasing this number will increase the sensitivity


            return true;

        }
        /*
        telemetry.addData("dist from color", dist);
        telemetry.addData("color",
                String.format("r:%d, g:%d, b:%d, a:%d",
                        mColorSensor.red(),
                        mColorSensor.green(),
                        mColorSensor.blue(),
                        mColorSensor.alpha()));

         */

        telemetry.addData("distance (mm)", mDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();

        return false;

    }
    public static void capStoneOut() {

        capstone.setPower(1);
    }
    public static void capStoneStop() {

        capstone.setPower(0);
    }
    public static void runServoDown() {
        foundation1.setPower(-power_foundation);
        foundation2.setPower(-power_foundation);
    }
    public static void runServo(double pow) {
        foundation1.setPower(pow);
        foundation2.setPower(pow);
    }
    public static void runServoUp() {
        foundation1.setPower( power_foundation);
        foundation2.setPower( power_foundation );
    }
    public static void stopServo() {
        foundation1.setPower(0);
        foundation2.setPower(0);
    }
    public static void pistonHome() {
        if ( Math.abs(piston1.getPosition()-home) > eps || Math.abs(piston2.getPosition()-home) > eps) {
            piston1.setPosition(home);
            piston2.setPosition(home);
        }
    }
    public static void pistonOpen() {
        if (piston1.getPosition() < open-eps || piston2.getPosition() < open-eps) {
            piston1.setPosition(open);
            piston2.setPosition(open);
        }
    }
    public static void pistonInc() {

        piston1.setPosition(piston1.getPosition() + .05);
        piston2.setPosition(piston1.getPosition() + .05);
    }
    public static void pistonDec() {
        piston1.setPosition(piston1.getPosition() - .05);
        piston2.setPosition(piston1.getPosition() - .05);
    }
    public static void pistonClose() {
        if (piston1.getPosition() > close+eps || piston2.getPosition() > close+eps) {
            piston1.setPosition(close);
            piston2.setPosition(close);
        }
    }

    public class AUTO {
        //could make something that keeps track of position or something
        LinearOpMode extra_funcs;
        int most_recent_position =0;
        final int wall_buffer = 1900;//distance from wall after you back up, applies to all times after
        //was previously 500 when it worked
        //you have grabbed the black, hopefully will allow the robot to avoid the other robot
        //final int block_distance = 1700; //Distance from the middle block to either the rigth or left block
        final int block_distance_x = 780;
        //final int block_forward_dist = 1600; //Distance to move foward while grabbing the block
        //final int line_dist_from_start = 1000;
        //final int foundation_dist = 5350;
        final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values
        /* Recognizes block using the camera, returns the result by chaning the value of most_recent_position */
        public void recognize_block() {

            long start = System.currentTimeMillis();
            int[] center = Robot.center();
            int pos = -1;
            telemetry.addData("pos x",center[0]);
            telemetry.addData("pos y",center[1]);
            telemetry.addData("pos:",most_recent_position);
            for (int i = 0; ranges.length > i; i++) {

                telemetry.addData("range:" +i, "low" + ranges[i][0] + " hi:" + ranges[i][1]);
                if (center[0] >= ranges[i][0] && center[0] <= ranges[i][1]) {
                    pos = i;
                    telemetry.addData("new position!","new!");
                }

            }
            if(pos != -1) {
                most_recent_position = pos;

                //confiedences not implemented yet
                //will need to read motion sensors or somethingd
                //probably don't program it unless its not working well or something
            }
            telemetry.addData("time taken to calculate (millis)",System.currentTimeMillis()-start);
            telemetry.update();
        }
        public void grabBlock(int move_from_wall,int block_distance, int block_forward_dist) {
            {

                Robot.resetTime();
                ElapsedTime pantTime = new ElapsedTime();
                while (!checkDone(500)) { //need to check if some are done
                    //Robot.setMotors(powers, 1);
                    Robot.pantagraphDown(pantTime);
                    //Robot.intake();
                }
                Robot.reset_pow();
            }
            //extra time to put pantagraph down
            moveDirMax(0,-1,0,move_from_wall,500,-1,0);
            if(most_recent_position == 2) { //then to the right
                moveDirMax(-1,0,0,(int) (block_distance_x/Math.sqrt(2)),2000,-1,0);
                moveDirMax(0,-1,0,(int) (block_distance/Math.sqrt(2)),3000,-1,0); //these don't work
                //maybe coudl slightly improve this, but not by much, sine its limited by the speed of the pantagraph

            }
            if(most_recent_position == 0) {
                moveDirMax(1,0,0,(int) (block_distance_x/Math.sqrt(2)),2000,-1,0);
                moveDirMax(0,-1,0,(int) (block_distance/Math.sqrt(2)),3000,-1,0);
                //moveOrient(1,-1,0,0,block_distance,1000); //these don't work

            }
            if (most_recent_position == 1) {
                //the distance will have to be shorter here
                //need to divide by sqrt(2), since this is a 45,45,90 triangle, and we want ot move
                //and equal amount in terms of y
                moveDirMax(0, -1, 0, (int) (block_distance / Math.sqrt(2)), 1000,-1,0);
            }


            {

                Robot.resetTime();
                double[] powers = motorPower.calcMotorsFull(0, -.6, 0);
                for (int i = 0; Robot.motors.length > i; i++) {
                    Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
                }
                Robot.setMotors(powers, 1);
                ElapsedTime pantTime = new ElapsedTime();
                while (!checkDone(3000)) { //need to check if some are done
                    Robot.setMotors(powers, 1);
                    Robot.pantagraphDown(pantTime);
                    Robot.intake(.80); //maybe .7 will be more relaible
                }
                Robot.reset_pow();
            }
            //now we have intaked the block, so lets quickly move back
            //notee: still have ot accoutn for the difference in x position caused by goign for different bloccks
            moveDirMax(0,1,0,(int) (block_forward_dist+(block_distance/Math.sqrt(2))-wall_buffer),4000);
        }
        /* MOVE TO FOUNDATION AFTER GRABBING BLOCK */
            //direction up-- 1 is left, -1 is right
        public void moveToFoundationY(int direction_up, int line_dist_from_start, int foundation_dist) {
            turnOrient(0,direction_up,1450);//doesnt need to be that long
            if(direction_up == 1) { //consider whether the motion before added to length or subtracted
                if (most_recent_position == 2) {
                    telemetry.addData("info","move longer distance");
                    telemetry.addData("info","move longer distance");
                    telemetry.update();
                    moveDirMax(0, -1, 0, (int) (line_dist_from_start + (block_distance_x / Math.sqrt(2))), 2000);

                }

                else if (most_recent_position == 0) {
                    moveDirMax(0, -1, 0, (int) (line_dist_from_start - (block_distance_x / Math.sqrt(2))), 2000);
                } else {
                    moveDirMax(0, -1, 0, (line_dist_from_start), 2000);
                }
            }
            else {
                if (most_recent_position ==0) {
                    moveDirMax(0,-1,0,(int) (line_dist_from_start+ (block_distance_x/Math.sqrt(2))),2000);

                }

                else if (most_recent_position == 2) {
                    moveDirMax(0,-1,0,(int) (line_dist_from_start- (block_distance_x/Math.sqrt(2)) ),2000);
                }
                else {
                    moveDirMax(0,-1,0, (line_dist_from_start ),2000);
                }
            }
            turnOrient(0,direction_up,500); //turn to ensure direction
            moveDirMax(0,-1,0,foundation_dist,5000,1,0);

        }
        public void moveBack(int orient_dir,int dist) {
            turnOrient(0,orient_dir,500);
            moveDirMax(0,1,0,dist,5000,-1,0);
        }
        /* MOVE TOWARDS THE FOUNDATION, FIRST QUICKLY, THEN SLOWLY*/

        public void moveToFoundationX(int foundation_forward, int foundation_slow) { //moves towards the foundation
            turnOrient(-1,0,1500); //turn towards foundation
            moveDirMax(0,-1,0,foundation_forward-300,3000); //move towards foundation
            //moveDirMax(0,-1,0,foundation_buff,3000,0,-1);
            moveDir(0,-.5,0,foundation_slow,1000); // a little bit more on the slow side, to ensur eit doesn't push it too much
            { //outtake for a second

                Robot.resetTime();

                ElapsedTime pantTime = new ElapsedTime();
                while (pantTime.milliseconds() < 1000  ) {


                    Robot.outtake(.9);
                    //if(pantTime.milliseconds() < 500) {
                    Robot.runServoDown(); //is this the right amount of time?
                    //}
                }
                Robot.resetTime();
                Robot.reset_pow();
            }
        }
        /* ORIENT ROBOT TO FOUNDATION, MOVE BACKWARDS, REORIENT, AND THEN MOVE AND GRAB THE FOUNDATION */

        public void orientToFoundation(int foundation_back, int foundation_buff) {
            moveDirMax(0,1,0,foundation_back,3000);

            turnOrient(1,0,2500);
            //moveDirMax(1,0,0,foundation_side,3000); //not sure if this is in the right direction
            //moveDirMax(1,0,0,2000,3000); //2000?

            moveDir(0,.5,0,foundation_back+foundation_buff,3000);
            { //outtake for a second

                Robot.resetTime();

                ElapsedTime pantTime = new ElapsedTime();
                while (pantTime.milliseconds() < 1500  ) { //one second less, since running it earlier
                    Robot.runServoDown();
                }
                Robot.reset_pow();
            }
        }
        public void moveFoundation(int foundation_pull) {
            moveFoundation(foundation_pull,0);
        }
        public void moveFoundation(int foundation_pull,int pant_pow) {
            moveDirMax(0,-1,0,foundation_pull,3000,pant_pow,0);
            turnOrient(0,-1,3000);
        }
        public void outtakeBlock(int time) {
            { //outtake for a second

                Robot.resetTime();

                ElapsedTime pantTime = new ElapsedTime();
                while (pantTime.milliseconds() < time  ) {


                    Robot.outtake(1);
                }
                Robot.resetTime();
                Robot.reset_pow();
            }
        }
        /* RUN FOR A CERTAIN AMOUNT OF TIME TEMPLATE
        {

            Robot.resetTime();
            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1000) {
                Robot.runServoUp();
            }
            Robot.resetTime();
            Robot.reset_pow();
        }

         */

        //Maybe add a park thing in here later
        /*******
        ROBOT AUTO FUNCTIONALITY
         *******/
        //Turns robot to speicifc angle
        AUTO(LinearOpMode reference) {
            extra_funcs = reference;
        }
        public void turnOrient(double gamepady, double gamepadx, int max_time) {
            turnOrient(gamepady,gamepadx,max_time,0,0,0);
        }
        public void turnOrient(double gamepady, double gamepadx, int max_time, double pant_pow, double in_pow, double servo_power) {

            Robot.resetTime();
            Robot.reset_pow();

            double rot = -gamepady;
            double rot2 = gamepadx;


            {
                double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
                double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
                double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
                double goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
                double nrot = 0;


                if (Math.abs(goal_rot_ang) > 0.20) { //amount goal angle needs to be off to actually turn in that direction

                    nrot = goal_rot_ang * (mag_rot); //rotation is the direction of rotation
                    //test value of /2.0 just ot
                    //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
                } else {

                    if (goal_rot_ang != 0) {
                        nrot = (Math.abs(goal_rot_ang) / goal_rot_ang) * (mag_rot) * (Math.abs(goal_rot_ang) + .25);
                    }
                    // nrot = nrot>0?.1 + nrot:-.1+nrot;
                }

                Robot.resetTime();



                for (int i = 0; Robot.motors.length > i; i++) {
                    Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //Robot.motors[i].setTargetPosition((int) ((Math.abs(powers[i]) / powers[i]) * max_dist)); //can amke this faster
                    //maybe rmeove the get arget position and see what happens, it if doesn't crash, then we just run for time
                }
                Robot.setMotors(0, 0, nrot, false);
            }
            double goal_rot_ang;
            {
                double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
                double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
                //double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
                goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
            }
            boolean change_sign  = false;
            boolean  prev_positive = (goal_rot_ang > 0);
            while( (! (Robot.timeElapsed() > max_time) && !change_sign) || Math.abs(goal_rot_ang) > .01) { //not sure if .05 is the rigth value, will have to test it a little

                double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
                double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
                double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
                goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
                double nrot = 0;

                if (prev_positive && goal_rot_ang < 0) {
                    change_sign = true;
                }
                if (!prev_positive && goal_rot_ang > 0) {
                    change_sign = true;
                }


                if (Math.abs(goal_rot_ang) > 0.20) { //amount goal angle needs to be off to actually turn in that direction

                    nrot = goal_rot_ang * (mag_rot ); //rotation is the direction of rotation
                    //test value of /2.0 just ot
                    //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
                } else {

                    //nrot = goal_rot_ang * (mag_rot) * (Math.abs(goal_rot_ang) + .4); //rotation is the direction of rotation
                    if (goal_rot_ang != 0) {
                        nrot = (Math.abs(goal_rot_ang) / goal_rot_ang) * (mag_rot) * (Math.abs(goal_rot_ang) + .25);
                    }
                    // nrot = nrot>0?.1 + nrot:-.1+nrot;
                }
                telemetry.addData("ang off",goal_rot_ang);
                //telemetry.update();

                telemetry.addData("time",max_time);
                telemetry.update();


                Robot.pantagraph.setPower(pant_pow);
                Robot.intakeR.setPower(in_pow);
                Robot.intakeL.setPower(in_pow);
                Robot.runServo(servo_power);
                Robot.setMotors(0, 0, nrot, Math.abs(goal_rot_ang) > 0.20);

            }
            Robot.reset_pow();
            Robot.resetTime();//reset these things at the end

            //sleep(150);


            //maybe want to turn in place, because it seems time might start ot b ea problem


        }
        //move and maintain direction (not working yet)
        public void moveOrient(double vx, double vy, double rot,int rot2, int dist, int max_time,double pant_pow,double in_pow) { //this probalby needs some work


            Robot.resetTime();
            double[] powers = Robot.orientMode(vx, vy, rot,rot2);
            double[] powers_orig = Robot.orientMode(vx, vy, rot,rot2);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

            }
            Robot.setMotors(powers_orig, 1);
            while (!checkDone(max_time)) { //need to check if some are done
                //powers = Robot.orientMode(vx, vy, rot,rot2);
            /*
            boolean can_change = false; //NOTE: SET TO FALSE FOR NOW
            //boolean can_change = true;
            for (int i = 0; powers.length > i; i++) {
                if ( (powers[i] == 0 || powers_orig[i] == 0) || (Math.abs(powers[i])/powers[i] != Math.abs(powers_orig[i])/powers_orig[i])  ) {
                    can_change = false;
                }
            }
            if(can_change) {
                Robot.setMotors(powers, 1); //This probably don't work properly
                //need to change the target direction probably
            }

             */
                Robot.setMotors(powers_orig, 1);
                Robot.pantagraph.setPower(pant_pow);
                Robot.intakeR.setPower(in_pow);
                Robot.intakeL.setPower(in_pow);

            }
            Robot.reset_pow();
            sleep(100); //lets get rid of this sleep for now

        }
        //NOTE, NEGATIVE FOR Y IS FORWARD!
        public void moveDir(double vx, double vy, double rot,int dist, int max_time) {

            moveDir(vx,vy,  rot, dist, max_time,0,0,0);
        }
        public void moveDir(double vx, double vy, double rot,int dist, int max_time, double pant_pow,double in_pow, double servo_power) {


            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

            }
            Robot.setMotors(powers, 1);
            while (!checkDone(max_time)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraph.setPower(pant_pow);
                Robot.intakeR.setPower(in_pow);
                Robot.intakeL.setPower(in_pow);
                Robot.runServo(servo_power);
            }
            Robot.reset_pow();
            sleep(100);
            Robot.resetTime();

        }
        public void moveDirMax(double vx, double vy, double rot,int dist, int max_time) {

            moveDirMax(vx,vy,rot,dist,max_time,0,0,0);
        }
        public void moveDirMax(double vx, double vy, double rot,int dist, int max_time, double pant_power, double in_pow) {

            moveDirMax(vx,vy,rot,dist,max_time,pant_power,in_pow,0);
        }
        public void moveDirMax(double vx, double vy, double rot,int dist, int max_time,double pant_pow,double in_pow,double servo_power) {


            Robot.resetTime();
            Robot.reset_pow();
            double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

            }
            Robot.setMotorsMax(powers, 1);
            while (!checkDone(max_time)) { //need to check if some are done
                Robot.setMotorsMax(powers, 1);
                Robot.pantagraph.setPower(pant_pow);
                Robot.intakeR.setPower(in_pow);
                Robot.intakeL.setPower(in_pow);
                Robot.runServo(servo_power);
            }
            Robot.reset_pow();
            sleep(100);
            Robot.resetTime();
        }
        public boolean checkDone(int maxTime) { //true if done
            //return true when done
            if (Robot.timeElapsed() > maxTime ) { //max amount of time alloted
                telemetry.addData("out of time","time ran out");
                return true;
            }
            if(extra_funcs.isStopRequested()) {
                Robot.reset_pow();
                throw new StopException();
                //should force it to stop
                //return true;
            }
            if(Robot.motorsFinished() >= 4 && Robot.timeElapsed() > 150) { //150 milli before it will end
                return true; //expiermental code here
            }
       /*
       if(Robot.motorsFinished() >= 2) {
           return true;
           //then done
       }

        */


            return false;
        }

        public void sleep(int maxtime) { //robot sleeps for a certain amoutn of time
            ElapsedTime time = new ElapsedTime();
            while(time.milliseconds() < maxtime) {
                //nothing
            }
            Robot.resetTime();


        }
    }
    public class StopException extends RuntimeException {
        public StopException() {super();}
    }




}
