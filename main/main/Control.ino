/*
void controlCinematico(){
      //Bpuesto= 90-60, Cpuesto=90 -(-20);
      //stepperY.move(convertirGrados(90-60));
      //stepperZ.move(convertirGrados(20));    
      controller.move(convertirGrados(0,2), 
             convertirGrados(90-60),
             convertirGrados(20));
      
      r = find_RadioC(Cpuesto);
      float B = find_B(Cpuesto, r);
      teta = extTeta(B);
      
      x = r*cos(grad2rad(teta));
      y = r*sin(grad2rad(teta));

      float yNuevo = SubidaVertical + y;
      float tetaNuevo = rad2Grad(atan(yNuevo/x));

      r = yNuevo/sin(grad2rad(tetaNuevo));

      //Cinematica

      float nuevoB = find_B2(r);
      Bpuesto = tetaNuevo+nuevoB;

      

      //Funcion que resta para darle la bajada o subida
      gradosDecorador[1] = int(90-Bpuesto);

      float nuevoC = find_C2(r, nuevoB);
      //Funcion que resta para darle la bajada o subida
      gradosDecorador[2] = int(90-nuevoC) * -1;
      
      Serial.println(r);
      Serial.println(B);
      Serial.println(teta);

      Serial.println(x);
      Serial.println(y);

      Serial.println(yNuevo);
      Serial.println(tetaNuevo);

      Serial.println(r);
      Serial.println(nuevoB);
    
      Serial.println(Bpuesto);
      Serial.println(nuevoC);
      
      delay(200);
      //stepperY.move(convertirGrados(-13));
      //stepperZ.move(convertirGrados(-27));
      controller.move(convertirGrados(0,2), 
             convertirGrados(-13),
             convertirGrados(-27));

}
*/

void cinematicaInv(int AnguloB, int AnguloC){
  Bpuesto = 90 - AnguloB;
  Cpuesto = 90 -(-AnguloC);

  //Serial.print("Bpuesto: ");  Serial.print(Bpuesto); 
  //Serial.print(" Cpuesto: ");  Serial.println(Cpuesto); 
  r = find_RadioC(Cpuesto);
      //Serial.println(r);
      
      float B = find_B(Cpuesto, r);
      //Serial.println(B);

      //Extraccion de coordenadas Polares y cordenadas Rectangulares
      teta = extTeta(B);
      //Serial.println(teta);
      
      x = r*cos(grad2rad(teta));
      y = r*sin(grad2rad(teta));
      //Serial.println(x);
      //Serial.println(y);

      //Esta es donde se entabla la nueva orden
      float yNuevo = SubidaVertical + y;
      float tetaNuevo = rad2Grad(atan(yNuevo/x));
      //Serial.println(yNuevo);
      //Serial.println(tetaNuevo);
      
      

      r = yNuevo/sin(grad2rad(tetaNuevo));
      //Serial.println(r);

      //Cinematica

      float nuevoB = find_B2(r);
      //Serial.println(nuevoB);

      float nuevoC = find_C2(r, nuevoB);
      //Serial.println(nuevoC);


      //Serial.print(" Bpuesto: ");  Serial.println(Bpuesto);
      //Serial.print(" tetaNuevo: ");  Serial.println(tetaNuevo);
      //Serial.print(" nuevoB: ");  Serial.println(nuevoB);
      
      float BpuestoNuevo = Bpuesto - (tetaNuevo + nuevoB);
      Serial.print("BpuestoN: ");  Serial.println(BpuestoNuevo); 
      
      //Serial.print(" Cpuesto: ");  Serial.println(Cpuesto);
      //Serial.print(" nuevoC: ");  Serial.println(nuevoC);
      float CpuestoNuevo = (180 - nuevoC) - Cpuesto;
      Serial.print("CpuestoN: ");  Serial.println(-1*CpuestoNuevo);
      
      outCinematicoB=BpuestoNuevo;
      outCinematicoC=CpuestoNuevo*-1;

      Serial.println(outCinematicoB);
      Serial.println(outCinematicoC);

      outCinematicoB= AnguloB+ outCinematicoB;
      outCinematicoC= AnguloC- outCinematicoC;

      Serial.println(outCinematicoB);
      Serial.println(outCinematicoC);

}
