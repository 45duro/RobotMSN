

void LecturaBotones(){
  
  LecturaBotonGuardarEEPROM = digitalRead(BotonGuardarEEPROM);
  
}

void button_on_press(GFButton & btn)
{
  // Use of the press count field
  Serial.print(F("Button pressed "));
  Serial.print(btn.getPressCount());
  Serial.println(F(" times."));
}

/**
   CALLBACK FOR BUTTON HOLD EVENT
*/
void button_on_hold(GFButton & btn){
  // Use only the first gold event
  if (btn.isFirstHold())
  {
    // Use of the getHoldTime() method
    Serial.println(F("Aplicando Cinematica"));
    LecturaBotonGuardar = 2;
  } else {
    // Print dots as long as the button is pressed
    Serial.print('.');
  }
}


/**
   CALLBACK FOR CLICK EVENT
*/
void button_on_click(GFButton & button)
{
  if (btn.getClicks()==2)
    LecturaBotonGuardar = 1;
}
