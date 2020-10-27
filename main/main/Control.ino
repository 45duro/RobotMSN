

void cinematicaInv(int AnguloB, int AnguloC){
  
  Bpuesto = 90 - AnguloB;
  Cpuesto = AnguloC;

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
      float CpuestoNuevo;

      if(AnguloC >= 90 ){
        CpuestoNuevo= 180 - nuevoC - Cpuesto;
        Serial.print("CpuestoN: ");  Serial.println(CpuestoNuevo);
      }
      else{
        CpuestoNuevo= nuevoC - Cpuesto;
        Serial.print("CpuestoN: ");  Serial.println(CpuestoNuevo);
      }
      
      outCinematicoB=BpuestoNuevo;
      outCinematicoC=CpuestoNuevo;
      
      Serial.println(outCinematicoB);
      Serial.println(outCinematicoC);
      
      outCinematicoB= AnguloB + outCinematicoB;
      outCinematicoC= AnguloC + outCinematicoC;

      Serial.println(outCinematicoB);
      Serial.println(outCinematicoC);

}


//movimientoJoyStick(valor A Pasar, Motor_para_accionar, vector donde se Guarda, Valor o rango de sensibilidad)
short movimientoJoyStick(short joy, byte pos, byte limCentral = 112){
    
    //Valores del joystick para evitar que se mueva en el centro o rango de NO movimiento cuando se suelta
    //El valor no entra linealizado es decir va de 0 a 1023
    if (joy < (512-limCentral) || joy > (512+limCentral)){
    
      //linealizo de -10 a 10 para la suavidad
      joy = map(joy,0,1023, -1, 1);
      //guardar en variable global
      grados[pos] += joy;

      return joy;
    }
    else{
      return 0;
    }
//      #if debug
//        Serial.print(joy); Serial.print("\t\t");  Serial.print(grados[pos]);
//      #endif

    

    

}
