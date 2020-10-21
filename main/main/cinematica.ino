float rad2Grad(float rad){
  return rad*180/phi;
}

float grad2rad(float grad){
  return grad*phi/180;
}



//Encuentra un radio principal que se carga cuando el programa inicia normalmente va a ser con radios de 90 para codo y 90 para base
float find_RadioC(float cPuesto){
  float flag;
  flag = pow(sizeBrazo, 2) + pow(sizeAnteBrazo, 2) - 2*(sizeAnteBrazo)*(sizeBrazo)*cos(grad2rad(cPuesto));
  flag = sqrt(flag);
  return flag;
}

//Encuentra la resultante del angulo B
float find_B(float cPuesto, float radioC){
  float flag;
  flag = (sin(grad2rad(cPuesto))*sizeBrazo)/radioC;
  flag = asin(flag);
  flag = rad2Grad(flag);
  return flag;
}

//Sacar el angulo de posicion Teta
float extTeta(float AnguloBeta){
  return Bpuesto-AnguloBeta;
}

//Sacar el angulo de posicion B segunda fase
double find_B2(float radioC){
  double flag;
  flag = pow(sizeAnteBrazo, 2) - pow(sizeBrazo, 2) - pow(radioC, 2);
  flag = flag/(-2 * sizeAnteBrazo * radioC );
  flag = acos(flag);
  flag = rad2Grad(flag);
  return flag;
}

//Sacar el angulo de posicion c 2da fase
float find_C2(float radio, float anguloB){
  float flag;
  flag = radio*sin(grad2rad(anguloB));
  flag /= sizeAnteBrazo;
  flag = asin(flag);
  return rad2Grad(flag);
}
