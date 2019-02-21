
#include <M5Stack.h>
#include <WiFi.h>
#include "HTTPResponses.h"
#include "Sensors.h"

/*
 * platformio lib install "M5Stack"
 * platformio init --ide <prefered ide> --board m5stack-fire
 */

const char *SSID = "M5Stack";
const char *PASSWORD = "12345678";
const int PORT = 80;

WiFiServer wifi_server(PORT);
IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

char *random_ssid_attachment = (char*)malloc(10);

String header;

const int FIRST_LINE_LAST_ACTIONS_Y_POS = 104;
const int FONT_HEIGHT = 16;

const int MAX_LENGTH_LAST_UPDATE_LIST = 5;

int current_length_last_update_list = 0;

String last_update_list[MAX_LENGTH_LAST_UPDATE_LIST] = {"", "", "", "", ""};


void setup()
{
    m5.begin();

    random_ssid_attachment[0] = '_';

    for(byte i = 0; i < 8; i++)
        random_ssid_attachment[i + 1] = random(0, 10) + 0x30;

    random_ssid_attachment[8 + 1] = '\0';

    char *ssid;
    ssid = (char *)malloc(strlen(SSID) + 1 + 9);

    strcpy(ssid, SSID);
    strcat(ssid, random_ssid_attachment);

    setup_mpu(); // needed for accelerometer and gyroscope; the magnetometer appears to be not working (yaw-values are weird)

    WiFi.softAP(ssid, PASSWORD);

    ip = WiFi.softAPIP();
    wifi_server.begin();

    m5.Lcd.setBrightness(100);

    m5.Lcd.setTextSize(3);
    m5.Lcd.print("Wi-Fi Server\n");

    m5.Lcd.setTextSize(2);
    m5.Lcd.print("SSID: ");
    m5.Lcd.print(ssid);
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

/*
 * used to document the last actions performed by the M5Stack
 * default is 5 based on the constant MAX_LENGTH_LAST_UPDATE_LIST
 */
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

/*
 * parses the user's request
 * returns the last entry of the url (e.g. /accel)
 */
String parseGET(String s)
{
    String temp;

    for(int i = 0, k = 0; i < s.length(); i++)
    {
        if(s[i] == ' ')
            k++;

        if (k == 1)
            if (s[i] != ' ')
                temp += s[i];

        if (k == 2)
            break;
    }

    return temp;
}

void loop()
{
    update_mpu(); // needed for accelerometer and gyroscope; the magnetometer appears to be not working (yaw-values are weird)

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
                        if (parseGET(currentLine) == "/gyro")
                        {
                            auto gyroscope = String(gyroscope_x()) + ", " + String(gyroscope_y()) + ", " + String(gyroscope_z());

                            client.println(BASE_HTML + text_div("Gyroscope", gyroscope) + HTML_END);

                            update_last_actions("gyro");

                            break;
                        }
                        else if (parseGET(currentLine) == "/accel")
                        {
                            auto accelerometer = String(accelerometer_x()) + ", " + String(accelerometer_y()) + ", " + String(accelerometer_z());

                            client.println(BASE_HTML + text_div("Accelerometer",     accelerometer) + HTML_END);
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

                            client.println(BASE_HTML + text_div("Sound", String("Sound ") + String("played")) + HTML_END);
                            update_last_actions("sound");

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
                            auto gyroscope = String(gyroscope_x()) + ", " + String(gyroscope_y()) + ", " + String(gyroscope_z());
                            auto accelerometer = String(accelerometer_x()) + ", " + String(accelerometer_y()) + ", " + String(accelerometer_z());

                            client.println(BASE_HTML + all_sensors_div(gyroscope, accelerometer) + HTML_END);

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
