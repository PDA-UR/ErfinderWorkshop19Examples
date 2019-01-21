
#include <M5Stack.h>
#include <WiFi.h>
#include "HTTPResponses.h"

/*
 * platformio lib install "M5Stack"
 * platformio init --ide <prefered ide> --board m5stack-fire
 * */

const char * SSID = "M5Stack";
const char * PASSWORD = "12345678";
const int PORT = 80;

WiFiServer wifi_server(PORT);
IPAddress ip(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

String header;

const int FIRST_LINE_LAST_ACTIONS_Y_POS = 104;
const int FONT_HEIGHT = 16;

const int LENGTH_LAST_UPDATE_LIST = 5;

String last_update_list[LENGTH_LAST_UPDATE_LIST] = {"", "", "", "", ""};

void setup()
{
    m5.begin();

    WiFi.softAP(SSID, PASSWORD);

    ip = WiFi.softAPIP();
    wifi_server.begin();

    m5.Lcd.setBrightness(100);
    m5.Lcd.setTextSize(3);
    m5.Lcd.print("Wi-Fi Server\n");
    m5.Lcd.setTextSize(2);
    m5.Lcd.print("SSID: ");
    m5.Lcd.print(SSID);
    m5.Lcd.print("\n");
    m5.Lcd.print("PASSWORD: ");
    m5.Lcd.print(PASSWORD);
    m5.Lcd.print("\n");
    m5.Lcd.print("URL: ");
    m5.Lcd.print(ip);
    m5.Lcd.print("\n");
    m5.Lcd.println();
    m5.Lcd.println("Last Actions:");
}

void update_last_actions(String last_action)
{
    for(int i = 1; i < LENGTH_LAST_UPDATE_LIST; i++)
        last_update_list[i - 1] = last_update_list[i];

    last_update_list[LENGTH_LAST_UPDATE_LIST - 1] = last_action;

    for (int i = 0; i < LENGTH_LAST_UPDATE_LIST; i++)
    {
        m5.Lcd.setCursor(0, FIRST_LINE_LAST_ACTIONS_Y_POS + FONT_HEIGHT * i);
        m5.Lcd.print("               ");  // 15x white space to erase a line
        m5.Lcd.setCursor(0, FIRST_LINE_LAST_ACTIONS_Y_POS + FONT_HEIGHT * i);
        m5.Lcd.println(last_update_list[i]);
    }
}

String parseGET(String s)
{
    String temp;

    for(int i = 0, k = 0; i < s.length(); i++)
    {
        if(s[i] == ' ') k++;

        if (k == 1)
            if (s[i] != ' ') temp += s[i];

        if (k == 2) break;
    }

    return temp;
}

void loop()
{
    WiFiClient client = wifi_server.available(); // listen for incoming clients

    if (client)
    {
        String currentLine = "";

        while (client.connected())
        {
            if (client.available())
            {
                char c = client.read();

                if (c == '\n')
                {
                    if (currentLine.length() == 0)
                    {
                        client.println(BASE_HTML + HTML_END);
                        break;
                    }
                    else
                    {
                        if (parseGET(currentLine) == "/temperature")
                        {
                            update_last_actions("temperature");

                            break;
                        }
                        else if (parseGET(currentLine) == "/gyro")
                        {
                            update_last_actions("gyro");

                            break;
                        }
                        else if (parseGET(currentLine) == "/compass")
                        {
                            update_last_actions("compass");

                            break;
                        }
                        else if (parseGET(currentLine) == "/accel")
                        {
                            update_last_actions("accel");

                            break;
                        }
                        else if (parseGET(currentLine) == "/buttons")
                        {
                            client.println(BASE_HTML + BUTTON_DIV +  HTML_END);
                            update_last_actions("buttons");

                            break;
                        }
                        else if (parseGET(currentLine) == "/sound")
                        {
                            update_last_actions("sound");

                            break;
                        }
                        else if (parseGET(currentLine) == "/image")
                        {
                            update_last_actions("image");

                            break;
                        }
                        else if (parseGET(currentLine) == "/demos")
                        {
                            update_last_actions("demos");

                            break;
                        }
                        else if (parseGET(currentLine) == "/all")
                        {
                            update_last_actions("all_sensors");

                            break;
                        }

                        currentLine = "";
                    }
                }
                else if (c != '\r')
                {
                    currentLine += c;
                }
            }
        }

        client.stop();
    }
}
