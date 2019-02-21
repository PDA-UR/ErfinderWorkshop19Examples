#include <M5Stack.h>
#include <WiFi.h>

#define LOOP_DELAY 100
#define MAX_MESSAGE_LENGTH 128

// globals for connection data
#define PORT 80
#define SSID "M5Stack" // change me!
#define PASSWORD "changeMe"

// stuff we need to set up the server
WiFiServer server(PORT);
WiFiClient client;
IPAddress IP(192, 168, 4, 1);
IPAddress GATEWAY(192, 168, 0, 1);
IPAddress SUBNET(255, 255, 255, 0);

// prints a message to the LCD screen
void handleMessage(char* message, int length)
{
    for(int i = 0; i < length; i++)
    {
        M5.Lcd.print(message[i]);
    }

    M5.Lcd.print("\n");
}

// prints the server's ip, ssid and password on the LCD
void printServerMessage()
{
    M5.Lcd.setCursor(50, 50);
    M5.Lcd.print("I'm a server!");

    M5.Lcd.setCursor(50, 70);
    M5.Lcd.print("IP: ");
    M5.Lcd.print(IP);

    M5.Lcd.setCursor(50, 90);
    M5.Lcd.print("SSID: ");
    M5.Lcd.print(SSID);

    M5.Lcd.setCursor(50, 110);
    M5.Lcd.print("Password: ");
    M5.Lcd.print(PASSWORD);

    M5.Lcd.setCursor(50, 130);
}

void setup()
{
    M5.begin(); // initialize M5Stack

    M5.Lcd.fillScreen(BLACK);

    // set up the server
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASSWORD);
    WiFi.begin();
    server.begin();

    printServerMessage(); // print ip, ssid and password on the builtin LCD
}

void loop()
{
    if(!client.connected())
    {
        // wait for a client and accept the connection
        client = server.available();

        if(client != 0) // a client wants to connect!
        {
            M5.Lcd.print("client: ");
            M5.Lcd.println(client);
            M5.Lcd.setCursor(0, 150);
        }

        delay(100);
        return;
    }
    else
    {
        if(client.available()) // there is data coming from the client
        {
            M5.Lcd.println("message from client!");

            int messageLength = 0; // length of the message we receive
            char* message = (char *) malloc(MAX_MESSAGE_LENGTH * sizeof(char)); // the actual message, limited to a certain amout of characters in length
            int buffer; // as we read the message one character at a time, we need a buffer to store the character

            // read until we reach the end of the message or the character limit
            while(messageLength < MAX_MESSAGE_LENGTH)
            {
                buffer = client.read(); // read one byte from the incoming message
                
                if(buffer == -1) break; // if there is no data, stop

                message[messageLength] = (char) buffer; // append the received character to the message

                messageLength++; // the length of our message has been increased by one
            }

            if(messageLength > 0)
            {
                // do stuff with the message here
                handleMessage(message, messageLength);
            }

            free(message); // we don't need the message anymore, so we can delete it from memory
        }
    }
    
    delay(LOOP_DELAY);
}
