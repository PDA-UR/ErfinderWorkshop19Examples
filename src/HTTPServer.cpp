
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

const int MAX_LENGTH_LAST_UPDATE_LIST = 5;

int current_length_last_update_list = 0;

String last_update_list[MAX_LENGTH_LAST_UPDATE_LIST] = {"", "", "", "", ""};


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

    m5.Speaker.begin();

}

void update_last_actions(String last_action)
{
    if (current_length_last_update_list < MAX_LENGTH_LAST_UPDATE_LIST)
        last_update_list[current_length_last_update_list++] = last_action;
    else
    {
        for (int i = 1; i < MAX_LENGTH_LAST_UPDATE_LIST; i++)
            last_update_list[i - 1] = last_update_list[i];

        last_update_list[MAX_LENGTH_LAST_UPDATE_LIST - 1] = last_action;
    }

    for (int i = 0; i < MAX_LENGTH_LAST_UPDATE_LIST; i++)
    {
        m5.Lcd.setCursor(0, FIRST_LINE_LAST_ACTIONS_Y_POS + FONT_HEIGHT * i);
        m5.Lcd.print("                  ");  // 18x white space to erase a line
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
                            // place function to retrieve temperature at this spot here v
                            // function must return a String with reasonable format     v
                            client.println(BASE_HTML + text_div("Temperature",     "25° Celsius") + HTML_END);
                            update_last_actions("temperature");

                            break;
                        }
                        else if (parseGET(currentLine) == "/gyro")
                        {
                            // place function to retrieve gyroscope at this spot here   v
                            // function must return a String with reasonable format     v
                            client.println(BASE_HTML + text_div("Gyroscope",       "(x, y, z)") + HTML_END);

                            update_last_actions("gyro");

                            break;
                        }
                        else if (parseGET(currentLine) == "/compass")
                        {
                            // place function to retrieve compass data at this spot here v
                            // function must return a String with reasonable format      v
                            client.println(BASE_HTML + text_div("Compass", /*regex:*/ "[N|NW|W|SW|S|SE|E|NE]") + HTML_END);

                            update_last_actions("compass");

                            break;
                        }
                        else if (parseGET(currentLine) == "/accel")
                        {
                            // place function to retrieve accelerometer at this spot here v
                            // function must return a String with reasonable format       v
                            client.println(BASE_HTML + text_div("Accelerometer",     "(x, y, z)") + HTML_END);
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
                            m5.Speaker.setBeep(30, 1000);
                            m5.Speaker.beep();

                            client.println(BASE_HTML + text_div("Sound", String("Notice me") + String("played")) + HTML_END);
                            update_last_actions("sound");

                            break;
                        }
                        else if (parseGET(currentLine) == "/image")
                        {
                            //                                                                                  Set your image alt text here v
                            //                                                                    Set your image height here v               v
                            //                                                                Set your image width here v    v               v
                            //   (i do not know if this is possible on M5 Stack) Set a valid path to an image here v    v    v               v
                            //                                   Set name of image here v                          v    v    v               v
                            client.println(BASE_HTML + text_div("Image", String("<Name of Image>") + "") + img_div("", 640, 480, "There should be an image.") + HTML_END);
                            update_last_actions("image");

                            break;
                        }
                        else if (parseGET(currentLine) == "/demos")
                        {
                            // probably buttons?

                            update_last_actions("demos");

                            break;
                        }
                        else if (parseGET(currentLine) == "/all")
                        {
                            //                                         temperature  gyroscope values     compass value     accelerometer values
                            client.println(BASE_HTML + all_sensors_div("25° Celsius", "(x, y, z)",  "[N|NW|W|SW|S|SE|E|NE]", "(x, y, z)") + HTML_END);

                            update_last_actions("all_sensors");
                            break;
                        }
                        else if (parseGET(currentLine) == "/buttons/a")
                        {
                            client.println(BASE_HTML + BUTTON_DIV +  HTML_END);
                            update_last_actions("A pressed Website");
                            break;
                        }
                        else if (parseGET(currentLine) == "/buttons/b")
                        {
                            client.println(BASE_HTML + BUTTON_DIV +  HTML_END);
                            update_last_actions("B pressed Website");
                            break;
                        }
                        else if (parseGET(currentLine) == "/buttons/c")
                        {
                            client.println(BASE_HTML + BUTTON_DIV +  HTML_END);
                            update_last_actions("C pressed Website");
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

    m5.update();
}
