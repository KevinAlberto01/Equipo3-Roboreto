//Longitud del brazo upArm
float F = 50;
//Longitude del brazo loArm (50) + longitud del brazo final (20)
float T = 70;
int prevX = posX;
int prevY = posY;
int prevZ = posZ;

void InverseKinematics(){
  int X = posX;
  int Y = posY;
  int Z = posZ;
  
  //Se asegura que el brazo no atraviese el piso
  if(Z > 62){
      Z = 62;
      posZ = Z;
  }
  
  //Calcula la magnitud del vector en xy
  float L = sqrt(Y*Y + X*X);
  //Calcula la magnitud del vector en xyz
  float d = sqrt(Z*Z+ L*L);
  
  //Se asegura que el brazo no se intente extender de más
  if(d <= 120){
    //Calcula el ángulo de la primera articulación
    gamma = atan2(Y, X);
    //Calcula el ángulo de la segunda articulación
    alpha = PI/2 - (atan2(L, Z) + acos((T*T - F*F - d*d)/(-2*F*d)));
    //Calcula el ángulo de la tercera articulación
    beta = -PI + acos((d*d - T*T - F*F) / (-2*F*T));
    
    prevX = X;
    prevY = Y;
    prevZ = Z;
  }
  else{
    posX = prevX;
    posY = prevY;
    posZ = prevZ;
  }
}

//Calcula los ángulos de las articulaciones
void writePos(){
  InverseKinematics();
}
