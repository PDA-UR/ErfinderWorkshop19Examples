//
// Created by juergen on 16/01/19.
//

#include <M5Stack.h>

/*
 * platformio lib install "M5Stack"
 * platformio init --ide <prefered ide> --board m5stack-fire
 * */

void setup()
{
    m5.begin();

    m5.Lcd.printf("Hello World");
}

void loop()
{

}