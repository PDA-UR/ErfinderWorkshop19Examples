
#ifndef LIBEWS_HTTPRESPONSES_H
#define LIBEWS_HTTPRESPONSES_H

#include <M5Stack.h>

String BASE_HTML =   "<!DOCTYPE html>\n"
                     "<html lang=\"en\">\n"
                     "<head>\n"
                     "    <meta charset=\"UTF-8\">\n"
                     "    <title>Title</title>\n"
                     "\n"
                     "    <style>\n"
                     "        .button {\n"
                     "            border: none;\n"
                     "            color: white;\n"
                     "            padding: 15px 32px;\n"
                     "            text-align: center;\n"
                     "            text-decoration: none;\n"
                     "            display: inline-block;\n"
                     "            font-size: 16px;\n"
                     "            margin: 4px 2px;\n"
                     "            cursor: pointer;\n"
                     "        }\n"
                     "\n"
                     "        .grid {\n"
                     "            display: flex;\n"
                     "            flex-wrap: wrap;\n"
                     "            justify-content: space-around;\n"
                     "        }\n"
                     "\n"
                     "        .cell {\n"
                     "            flex: 0 0 32%;\n"
                     "            height: 50px;\n"
                     "            margin-bottom: 5px;\n"
                     "            background-color: #999;\n"
                     "        }\n"
                     "\n"
                     "    </style>\n"
                     "</head>\n"
                     "<body>\n"
                     "    <h1 align=\"center\">M5 Monitor</h1>\n"
                     "\n"
                     "    <div class=\"grid\">\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/temperature';\" value=\"M5 Temperature\">M5 Temperature</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/gyro';\" value=\"M5 Gyroscope\">M5 Gyroscope</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/compass';\" value=\"M5 Compass\">M5 Compass</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/accel';\" value=\"M5 Accelerometer\">M5 Accelerometer</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/buttons';\" value=\"M5 Buttons\">M5 Buttons</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/sound';\" value=\"M5 Sound\">M5 Sound</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/image';\" value=\"M5 Image\">M5 Image</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/demos';\" value=\"M5 Demos\">M5 Demos</button></button>\n"
                     "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/all';\" value=\"M5 Show all Sensors\">M5 Show all Sensors</button></button>\n"
                     "    </div>\n";

String HTML_END = "</body>\n"
                        "</html";

String BUTTON_DIV = "<h1 align=\"center\">M5 Buttons</h1>\n"
                    "\n"
                    "    <div class=\"grid\" id=\"m5 buttons\">\n"
                    "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/temperature';\" value=\"M5 Temperature\">M5 Button A</button></button>\n"
                    "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/gyro';\" value=\"M5 Gyroscope\">M5 Button B</button></button>\n"
                    "        <button class=\"button cell\" type=\"button\" onclick=\"location.href='http://192.168.4.1/compass';\" value=\"M5 Compass\">M5 Button C</button></button>\n"
                    "    </div>";

#endif //LIBEWS_HTTPRESPONSES_H
