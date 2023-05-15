class Crate{
  PVector position_c;
  PVector prev_position;
  boolean crash;
  
  float w = 20, h = 20, d = 20;
  float rotation;
  int floor = -68;
  float minDist = (w+h+d)/(3);
  
  //Constructor de la clase Crate: toma la posición con el marco de referencia de Processing
  Crate(float x_c, float yT_c, float z_c, float rY_c){
    float y_c = yT_c + floor + h/2;
    position_c = new PVector(x_c, y_c, z_c);
    rotation = rY_c;
  }

  //Modifica la posición de la caja
  void update(int xCol, int yCol, int zCol){
    int xP = -yCol;
    int yP = -zCol;
    int zP = -xCol;
    position_c.x = xP;
    position_c.y = yP;
    position_c.z = zP;
  }
  
  //Revisa colisiones con el brazo y regresa un valor booleano
  boolean checkCollisions(int xCol, int yCol, int zCol){
    //Traduce del marco de referencia del robot a la referencia de processing
    int xP = -yCol;
    int yP = -zCol;
    int zP = -xCol;
    
    //Calcula la distancia entre el fin del brazo y el centro de la caja y
    //lo compara con un mínimo para considerar que se están tocando
    PVector distanceVect = new PVector(position_c.x - xP, position_c.y - yP, position_c.z - zP);
    float distMag = distanceVect.mag();
    if(distMag < minDist){
      return true;
    }
    else{
      return false;
    }
  }
  
  //Revisa colisiones con otras cajas
  boolean checkCrateCollisions(Crate other){
    PVector distanceVect = PVector.sub(other.position_c, position_c);
    float distMag = distanceVect.mag();
    
    //Si las cajas se están tocando evita que se muevan
    if(distMag <= minDist){
      position_c = prev_position;
      return crash = true;
    }
    else return crash = false;
  }
  
  //Si la caja no la está agarrando el brazo ni está en contacto con otra
  //cae hasta que choca con el piso
  void fall(){
    if(!crash && !grab && position_c.y > floor + h/2){
      position_c.y -= 0.1*(millisOld - gTime);
    }
  }
  
  //Ejecuta las funciones internas y despliega la imagen de la caja
  void display(){
    //Se asegura que la caja no caiga debajo del piso
    if(position_c.y < floor + h/2){
      position_c.y = floor + h/2;
    }
    
    fill(#402F1D);
    pushMatrix();
    translate(position_c.x, position_c.y, position_c.z);
    rotateY(rotation);
    box(w, h, d);
    popMatrix();
    prev_position = position_c.copy();
  }
}
