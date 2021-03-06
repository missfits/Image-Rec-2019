/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Size;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static UsbCamera startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    CameraServer inst = CameraServer.getInstance();
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = inst.startAutomaticCapture(camera);
    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    public Mat outputImg;
    //image size is 120 rows, 160 columns
    @Override
    public void process(Mat inputImg) {
      outputImg = new Mat(inputImg.rows(),inputImg.cols(), inputImg.type());
      Mat circles = new Mat();
      Mat gray = new Mat();

      Imgproc.cvtColor(inputImg,gray,Imgproc.COLOR_BGR2GRAY);
      Imgproc.medianBlur(gray, gray, 5);

      outputImg = inputImg;
      //(input image, output image, process (don't touch),inverse ratio, min distance between centers, edge detector threshold 1, edge detector threshold 2)
      Imgproc.HoughCircles(gray,circles, Imgproc.HOUGH_GRADIENT, 1, 25, 100, 30);
      //System.out.println("Circles: " + circles.cols());
      for(int a = 0; a < circles.cols(); a++){
        double[] c = circles.get(0,a);
        Point center = new Point(Math.round(c[0]),Math.round(c[1]));
        Imgproc.circle(outputImg,center,(int)Math.round(c[2]),new Scalar(0,0,255),2);
      }
    }
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }
    
    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }
    
    // start cameras
    List<UsbCamera> cameras = new ArrayList<>();
    for (CameraConfig cameraConfig : cameraConfigs) {
      cameras.add(startCamera(cameraConfig));
    }

    ntinst.getTable("RaspberryPi").addEntryListener("Vision Mode",(table,key,entry,value,flags) ->{
      System.out.println("Changing Exposure " + value.getBoolean());
      if(value.getBoolean()){
        for(UsbCamera camera : cameras){
          camera.setExposureManual(0);
          camera.getProperty("raw_exposure_absolute").set(10);
        }
      }else{
        for(UsbCamera camera : cameras){
          camera.setExposureAuto();
        }
      }
    }, EntryListenerFlags.kNew |  EntryListenerFlags.kUpdate);
    //TEMPORARY
    cameras.get(0).getProperty("raw_exposure_absolute").set(10);
    CvSource outputStream = CameraServer.getInstance().putVideo("Test", 416, 240); 
    //MjpegServer chosenCamera = CameraServer.getInstance().addSwitchedCamera("Jacob");
    // start image processing on camera 0 if present
    if (cameras.size() >= 2) {
      BetterVisionRunner<GripPipeline> runner = new BetterVisionRunner<GripPipeline>(cameras.get(1),cameras.get(0),new GripPipeline(),pipeline -> {
          //System.out.println("Vision Mode: " + ntinst.getTable("RaspberryPi").getEntry("Vision Mode").getBoolean(false));
          outputStream.putFrame(pipeline.outputImg);

          ntinst.getTable("RaspberryPi").getEntry("Center Offset").setNumber(pipeline.midOffset);
          //ntinst.getTable("RaspberryPi").getEntry("Side Offset").setNumber(pipeline.sideOffset);
      });
      Thread visionThread = new Thread(runner :: runForever);
     
      /*VisionThread visionThread2 = new VisionThread(cameras.get(0),
              new GripPipeline(), pipeline -> {
                outputStream.putFrame(pipeline.outputImg);
                  if(ntinst.getTable("RaspberryPi").getEntry("Vision Mode").getBoolean(false)){
                    cameras.get(0).setExposureManual(10);
                  }else{
                    cameras.get(0).setExposureAuto();
                  }
                ntinst.getTable("RaspberryPi").getEntry("Center Offset").setNumber(pipeline.midOffset);
      });*/
      
      visionThread.start();
      //visionThread2.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
