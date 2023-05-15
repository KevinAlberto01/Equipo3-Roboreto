/*
 Código principal para la teleoperación de un brazo robótico con
 el uso de gestos hechos con la mano captados con la webcam de la computadora.
 */
 
import gab.opencv.*;
import processing.video.Capture;
import java.awt.*;
import processing.core.PApplet;

PShape base, shoulder, upArm, loArm, end;
float rotX, rotY;
int posX = 0, posY = 50, posZ = 50;
float alpha , beta, gamma;
float millisOld, gTime, gSpeed = 4;
boolean newletter, grab = false, crash;
int[] pos = {0, 0, 0};
int prevFist = 0;

CameraApplet Ca;
Capture video;
OpenCV opencv;  

//Declara las cajas con sus posiciones y rotaciones
Crate[] crates = {
  new Crate(65, 0, -65, 0),
  new Crate(39, 0, -29, 0),
  new Crate(-27, 0, 18, 0),
  new Crate(70, 0, 60, 0),
  new Crate(-15, 0, 53, 0),
  new Crate(-20, 0, -29, 0),
  new Crate(-58, 0, -56, 0),
  new Crate(-83, 0, 46, 0),
  new Crate(29, 0, 35, 0),
  new Crate(0, 0, -67, 0)
  };
int[] crashCount = new int[crates.length];
boolean[] crashed = new boolean[crates.length];

void setup() {
  //Establece las características de la ventana (tamaño, espacio 3D)
  size(1200, 800, P3D);
  
  //Carga cada parte del modelo 3D del robot
  base = loadShape("r5.obj");
  shoulder = loadShape("r1.obj");
  upArm = loadShape("r2.obj");
  loArm = loadShape("r3.obj");
  end = loadShape("r4.obj");
  
  //Le permite a processing modificar el aspecto de los objetos
  shoulder.disableStyle();
  upArm.disableStyle();
  loArm.disableStyle();
  
  //Inicia la cámara para que se abra a la par que la ventana de la simulación
  String[] args = {"Camera"};
  video = new Capture(this, 640/2, 480/2);
  opencv = new OpenCV(this, 640/2, 480/2);
  Ca = new CameraApplet();
  PApplet.runSketch(args, Ca);
}

void draw() {
  //Interpreta los gestos captados por la cámara y los traduce a cambios en las coordenadas del robot
  if(Ca.fist == 1 && prevFist == 0){
    println("fist");
    grab = !grab;
  }
  if(Ca.palm == 1){
    println("palm");
    if(Ca.pCoord.x < 0 && Ca.pCoord.y > 0){
      pos[1] = -1;
    }
    else if(Ca.pCoord.x < 0 && Ca.pCoord.y < 0){
      pos[1] = 1;
    }
     else if(Ca.pCoord.x > 0 && Ca.pCoord.y > 0){
      pos[2] = -1;
    }
     else if(Ca.pCoord.x > 0 && Ca.pCoord.y < 0){
      pos[2] = 1;
    }
  }
  if(Ca.right == 1){
    println("right");
    if(Ca.pCoord.y > 0){
      pos[0] = -1;
    }
    else if(Ca.pCoord.y < 0){
      pos[0] = 1;
    }
  }
  prevFist = Ca.fist;
  
  //Llama cambia la posición de los ángulos de cada articulación
  writePos();
  setTime();
  
  //Establece el color del fondo, la iluminación y características visuales
  background(31);
  smooth();
  lights();
  directionalLight(51, 102, 126, -1, 0, 0);
  fill(#FFE308);
  noStroke();
  
  //Establece el marco de referencia como el origen de la ventana
  translate(width/2, height/2, 0);
  
  //Tamaño del modelo
  scale(-4);
  //Permite rotar la imagen
  rotateX(rotX);
  rotateY(-rotY);
  
  //Dibuja el suelo
  pushMatrix();
  translate(0,-68,0);
  box(300,2,300);
  popMatrix();
  
  pushMatrix();
  //Coloca la referencia de la base
  translate(0,-40,0);
    shape(base);
  
  //Coloca la referencia de la primera articulación
  translate(0,4,0);
  rotateY(gamma);
    shape(shoulder);
    
  //Coloca la referencia de la segunda articulación
  translate(0, 25, 0);
  rotateY(PI);
  rotateX(alpha);
    shape(upArm);
    
  //Coloca la referencia de la tercera articulación
  translate(0,0,50);
  rotateY(PI);
  rotateX(beta);
    shape(loArm);
    
  //Coloca la referencia del último componente
  translate(0, 0, -50);
  rotateY(PI);
    shape(end);
  popMatrix();
    
  //Dibuja cajas
  for(int i = 0; i<crates.length; i++){
    crates[i].display();
    //Revisa si el brazo está tocando la caja
    if(crates[i].checkCollisions(posX, posY, posZ) && grab){
      crates[i].update(posX, posY, posZ);
    }
    //Revisa colisiones con el resto de las cajas
    crashCount[i] = 0;
    for(int j = 0; j<crates.length; j++){
      if (j != i){
        crash = crates[i].checkCrateCollisions(crates[j]);
        if(crash){
          crashed[i] = true;
          crashCount[i] += 1;
        }
      }
    }
    if (crashCount[i] == 0){ crashed[i] = false;}
    if(!crashed[i]){
      crates[i].fall();
    }
  }
  //Modifica la posición del robot cuando se activa el trigger de movimiento
  if(newletter || Ca.hand){
    posX += pos[0];
    posY += pos[1];
    posZ += pos[2];
  }else if (!Ca.hand){
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0;
  }
  
  //Imprime las coordenadas del brazo
  //print(posX);
  //print(" : ");
  //print(posY);
  //print(" : ");
  //print(posZ);
  //print("\n");
  
}

//Permite rotar la imagen
void mouseDragged(){
  rotY -= (mouseX - pmouseX) * 0.01;
  rotX -= (mouseY - pmouseY) * 0.01;

}

//Controla el robot con el teclado (testing)
void keyPressed(){
  switch(key){
    case 'w':
    case 'W':
      pos[0] = -1;
      break;
    case 's':
    case 'S':
      pos[0] = 1;
      break;
    case 'a':
    case 'A':
      pos[1] = -1;
      break;
    case 'd':
    case 'D':
      pos[1] = 1;
      break;
    case 'z':
    case 'Z':
      pos[2] = -1;
      break;
    case 'x':
    case 'X':
      pos[2] = 1;
      break;
      
    case 'e':
    case 'E':
      grab = true;
      break;
      
    case 'q':
    case 'Q':
      grab = false;
      break;
  }
  newletter = true;
}

//Mantiene el movimiento hasta que se suelta la tecla
void keyReleased(){
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
  newletter = false;
}

//Calcula el tiempo actual
void setTime(){
  gTime += ((float)millis()/1000 - millisOld)*(gSpeed/4);
  if(gTime >= 4){
    gTime = 0;
  }
  millisOld = (float)millis()/1000;
}
