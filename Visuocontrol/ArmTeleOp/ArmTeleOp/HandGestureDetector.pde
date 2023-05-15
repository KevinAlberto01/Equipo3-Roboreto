import gab.opencv.*;
import processing.video.Capture;
import java.awt.*;
import processing.core.PApplet;

public class CameraApplet extends PApplet {
  
  int palm, fist, right, x, y;
  PVector pCoord = new PVector(x, y);
  PVector rCoord = new PVector(x, y);
  boolean hand;

  public void settings() {
    size(640, 480);
  }
  
  public void setup() {
    video = new Capture(this, 640/2, 480/2);
    opencv = new OpenCV(this, 640/2, 480/2);
    
    //opencv.loadCascade("aGest.xml");
    video.start();
  }
  
  public void draw() {
    scale(2);
    opencv.loadImage(video);
  
    translate(video.width, 0 );
    scale( -1, 1 );
    image( video, 0, 0 );
  
    noFill();
    stroke(0, 255, 0);
    strokeWeight(3);
    
    fist = detectFist();
    palm = detectPalm();
    right = detectRight();
    
    if(fist == 1 || palm ==1 || right == 1){
      hand = true;
    }
    else{
      hand = false;
    }
  }
  
  public void captureEvent(Capture c) {
    c.read();
  }
  
  public int detectFist(){
    opencv.loadCascade("fist.xml");
    
    Rectangle[] fist = opencv.detect();
    //println(fist.length);
  
    for (int i = 0; i < fist.length; i++) {
      //println(fist[i].x + "," + fist[i].y);
      rect(fist[i].x, fist[i].y, fist[i].width, fist[i].height);
    }
    return fist.length;
  }
  public int detectPalm(){
    opencv.loadCascade("rpalm.xml");
    
    Rectangle[] palm = opencv.detect();
    //println(palm.length);
  
    for (int i = 0; i < palm.length; i++) {
      //println(palm[i].x + "," + palm[i].y);
      pCoord.x = -1*(palm[i].x - 120);
      pCoord.y = -1*(palm[i].y - 65);
      println(pCoord.x + "," + pCoord.y);
      rect(palm[i].x, palm[i].y, palm[i].width, palm[i].height);
    }
    
    
    return palm.length;
  }
  public int detectRight(){
    opencv.loadCascade("right.xml");
    
    Rectangle[] right = opencv.detect();
    //println(palm.length);
  
    for (int i = 0; i < right.length; i++) {
      //println(right[i].x + "," + right[i].y);
      //pCoord.x = -1*(right[i].x - 120);
      pCoord.y = -1*(right[i].y - 65);
      rect(right[i].x, right[i].y, right[i].width, right[i].height);
    }
    
    return right.length;
  }
}
