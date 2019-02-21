#include <M5Stack.h>
#include <WiFi.h>

#define LOOP_DELAY 1000

// globals for connection data
#define PORT 80
#define SSID "M5Stack" // change me!
#define PASSWORD "changeMe"

// stuff we need to set up the connection
WiFiClient client;
IPAddress IP(192, 168, 4, 1);

void setup()
{
    M5.begin(); // initialize M5Stack
    
    M5.Lcd.fillScreen(BLACK);

    // set up the wifi connection
	WiFi.mode(WIFI_STA); 
	WiFi.begin(SSID, PASSWORD);
	
    M5.Lcd.setCursor(50, 50);
    M5.Lcd.println("I'm a client!");

	while(WiFi.status() != WL_CONNECTED)
	{
	    delay(100);
	}

    M5.Lcd.setCursor(50, 70);
    M5.Lcd.println("connecting...");
    M5.Lcd.setCursor(50, 90);

    // connect to the server
    while(!client.connected())
    {
        M5.Lcd.print(".");
        client.connect(IP, PORT);
        delay(100);
    }

    M5.Lcd.setCursor(0, 110);
}

// send stuff to the server
void loop()
{
    M5.Lcd.println("sendig: Hallo");
    client.write("Hallo!", 6); // send the message "Hallo!" with length 6
    delay(LOOP_DELAY);
}

