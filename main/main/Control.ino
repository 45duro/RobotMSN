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
