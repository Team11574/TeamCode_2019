package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name="Camera", group="Iterative Opmode")
public class Camera extends OpMode {
    public static WebcamName webCam;
    public static VuforiaLocalizer vuforia;

    @Override
    public void init() {
        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vu_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        vu_parameters.vuforiaLicenseKey = "AV29AFb/////AAABma3tuKm8DE2/tKJA0LIvwcIWOzMsiVbx8yLAiSRl1l98p84lwbzzJMkqsJw7ySFusaR6sYtQoSN9rzPIjUVqJ/uLkqv/V0rllY9LtZS0bnUfiyYarG+ZIDk587QhB/+BdT2EMo7w7+dHPO3Y9YOoFMZom016W6kYU+Tc7/OaN0AMXb6zGal02KRH3h913F+84o7J48sKXz0whgL1TSbfFQvYYyzijQlqzsmcvee4e3AI3L30L9AM1+COMhKcsIuYjpuUl1/oELl6XSCC7Q3UVnrKnah1WQb2C8m1KdsGgPbPp42rFC4ArXydJI193CEEENY/fyHvxxh8/aEb4fxxmybXkPk93BVpPZL6co8hFpSF";


        vu_parameters.cameraName = webCam;


        vuforia = ClassFactory.getInstance().createVuforia(vu_parameters);
        //
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        //GGGRRRRR BBBBBGGG
        //

    }

    @Override
    public void loop() {
    int[][] rgb = (int[][]) (readImage(readCamera())[0]);

    //maybe save this to a file somehwere

    }

    public static Object[] readImage(Image image) {
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
            long numImages = frame.getNumImages();
            for (int i = 0; numImages > i; i++) {
                if(frame.getImage(i).getFormat()==PIXEL_FORMAT.RGB888) {
                    rgb = frame.getImage(i);
                    //should break after this?
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return rgb;
    }

    //webCam = hardwareMap.get(WebcamName.class, "Webcam 1");

}
