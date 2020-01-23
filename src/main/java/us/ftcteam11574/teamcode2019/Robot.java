package us.ftcteam11574.teamcode2019;



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
    //everythign is public so it is easily accessible
        //this is, though, not the best coding practice
        //but it will make things easier for debugging
        //can always refactor the code later
    //everythign static because we don't want mroe than 1 robot
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
    public static double power_foundation = 1; //start at the max power


    // /* CAMERA
    public static WebcamName webCam;


    public static VuforiaLocalizer vuforia;





    public static void init_teleOp(Telemetry telemetry_, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        overallInit(telemetry_,hardwareMap_,gamepad1_,gamepad2_);

        //webCam
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
        //maye say were going to use encoders first

        for (int i = 0; motors.length > i; i++) {

            //motors[i].setPower(0);
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

            //Don't initilaize,since its not working right now
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


        //rgb_format_worked = vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.RGB888)[0];

        telemetry.addData("test","test");
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
            //int read2 = ((Byte)pixels.get()).intValue();



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
    public static void runServoDown() {
        foundation1.setPower(-power_foundation);
        foundation2.setPower(-power_foundation);
    }
    public static void runServoUp() {
        foundation1.setPower( power_foundation);
        foundation2.setPower( power_foundation );
    }
    public static void stopServo() {
        foundation1.setPower(0);
        foundation2.setPower(0 );
    }



}
