import gab.opencv.*;
import processing.video.Capture;
import java.awt.*;
import processing.core.PApplet;

//Declara el uso de la cámara y la identificación de gestos
//como un applet que se correrá al mismo tiempo que la simulación
public class CameraApplet extends PApplet {
  
  int palm, fist, right, x, y;
  PVector pCoord = new PVector(x, y);
  PVector rCoord = new PVector(x, y);
  boolean hand;

  //Declara las características de la ventana
  public void settings() {
    size(640, 480);
  }
  
  public void setup() {
    //Inicia la cámara y OpenCV
    video = new Capture(this, 640/2, 480/2);
    opencv = new OpenCV(this, 640/2, 480/2);
    
    video.start();
  }
  
  public void draw() {
    //Carga el frame actual al objeto de OpenCV, le hace un efecto espejo y lo muestra en la pantalla
    scale(2);
    opencv.loadImage(video);
  
    translate(video.width, 0 );
    scale( -1, 1 );
    image( video, 0, 0 );
  
    noFill();
    stroke(0, 255, 0);
    strokeWeight(3);
    
    //Ejecuta las funciones de identificación de gestos
    fist = detectFist();
    palm = detectPalm();
    right = detectRight();
    
    //Actualiza el trigger
    if(fist == 1 || palm ==1 || right == 1){
      hand = true;
    }
    else{
      hand = false;
    }
  }
  
  //Parmite leer la imagen de la cámara
  public void captureEvent(Capture c) {
    c.read();
  }
  
  //Función que detecta un puño y lo enmarca en la imagen según sus coordenadas
  public int detectFist(){
    //Abre el archivo cascade para detectar un puño
    //Este archivo debe colocarse dentro de las librerías de OpenCV dentro de Processing
    opencv.loadCascade("fist.xml");
    
    Rectangle[] fist = opencv.detect();
  
    for (int i = 0; i < fist.length; i++) {
      rect(fist[i].x, fist[i].y, fist[i].width, fist[i].height);
    }
    
    return fist.length;
  }
  
  //Función que detecta una mano abierta, la enmarca y calcula sus coordenadas
  public int detectPalm(){
    //Abre el archivo cascade para detectar una mano abierta
    //Este archivo debe colocarse dentro de las librerías de OpenCV dentro de Processing
    opencv.loadCascade("rpalm.xml");
    
    Rectangle[] palm = opencv.detect();
  
    for (int i = 0; i < palm.length; i++) {
      //Se calculan las coordenadas tomando como origen el centro de la imagen
      pCoord.x = -1*(palm[i].x - 120);
      pCoord.y = -1*(palm[i].y - 65);
      println(pCoord.x + "," + pCoord.y);
      rect(palm[i].x, palm[i].y, palm[i].width, palm[i].height);
    }
    
    return palm.length;
  }
  
  //Función que detecta una mano abierta girada a la derecha, la enmarca y calcula su coordenada en y
  public int detectRight(){
    //Abre el archivo cascade para detectar una mano abierta girada a la derecha
    //Este archivo debe colocarse dentro de las librerías de OpenCV dentro de Processing
    opencv.loadCascade("right.xml");
    
    Rectangle[] right = opencv.detect();
  
    for (int i = 0; i < right.length; i++) {
      //Se calculan las coordenadas tomando como origen el centro de la imagen
      pCoord.y = -1*(right[i].y - 65);
      rect(right[i].x, right[i].y, right[i].width, right[i].height);
    }
    
    return right.length;
  }
}
