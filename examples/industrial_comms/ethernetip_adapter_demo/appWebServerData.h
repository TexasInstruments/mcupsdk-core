/*!
 *  \example appWebServerData.h
 *
 *  \brief
 *  Application Web Server File Data
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-05-19
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef APP_WEBSRV_DATA
#define APP_WEBSRV_DATA

#ifdef __cplusplus
extern "C" {
#endif

	static const char response_200_content_html[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";

	static const char response_200_content_js[] = "HTTP/1.1 200 OK\r\nContent-type: text/javascript\r\n\r\n";

    static const char response_200_content_image[] = "HTTP/1.1 200 OK\r\nContent-type: image/x-icon\r\n\r\n";

    static const char response_404[] = "HTTP/1.1 404 Not Found\r\nContent-type: text/html\r\n\r\n"
    "<html>"
        "<head>"
            "<style>"
                "h1 {"
                    "text-align: center;"
                    "font-family: Arial,sans-serif;"
                "}"
                "h2 {"
                    "text-align: center;"
                    "font-family: Arial,sans-serif;"
                "}"
            "</style>"
            "<meta charset=\"utf-8\"/>"
            "<title>EtherNet/IP&trade; Adapter Web Server Example</title>"
        "</head>"
        "<body>"
            "<h1>EtherNet/IP&trade; Adapter Web Server Example</h1>"
            "<h2>404 : Not Found </h2>"
        "</body>"
    "</html>";

    static const char response_501[] = "HTTP/1.1 501 Not Implemented\r\nContent-type: text/html\r\n\r\n"
    "<html>"
        "<head>"
            "<style>"
                "h1 {"
                    "text-align: center;"
                    "font-family: Arial,sans-serif;"
                "}"
                "h2 {"
                    "text-align: center;"
                    "font-family: Arial,sans-serif;"
                "}"
            "</style>"
            "<meta charset=\"utf-8\"/>"
            "<title>EtherNet/IP&trade; Adapter Web Server Example</title>"
        "</head>"
        "<body>"
            "<h1>EtherNet/IP&trade; Adapter Web Server Example</h1>"
            "<h2>501 : Not Implemented </h2>"
        "</body>"
    "</html>";

    static const char main_html[] =
    "<!DOCTYPE HTML>"
    "<html>"
        "<head>"
            "<style>"
                "body {"
                    "margin: 0;"
                    "color: black;"
                    "background-color: white;"
                "}"
                "body, table, div, p, dl {"
                    "font: 400 14px/22px Roboto,sans-serif;"
                "}"
                "#titlearea {"
                    "padding: 0px;"
                    "margin: 0px;"
                    "width: 100%;"
                    "border-bottom: 0px;"
                "}"
                "#projectlogo {"
                    "border: 0px none;"
                    "vertical-align: middle;"
                    "padding-left: 20px;"
                    "padding-right: 40px;"
                    "padding-top: 0px;"
                    "padding-bottom: 0px;"
                    "height: 24px;"
                "}"
                "#projectalign {"
                    "vertical-align: middle;"
                "}"
                "#projectname {"
                    "font: 150% Arial,sans-serif;"
                    "padding-left: 0.5em;"
                    "margin: 0px;"
                "}"
                "#nav {"
                    "background: #c00;"
                    "height: 30px;"
                "}"
                "#content {"
                    "margin-top: 10px;"
                    "margin-left: 12px;"
                    "margin-right: 8px;"
                "}"
                "#threadinfo {"
                    "border: thin solid #2D4068;"
                    "border-collapse: collapse;"
                    "text-align: left;"
                "}"
                "#threadinfo caption {"
                    "border: none;"
                    "text-align: left;"
                    "font-weight: bold;"
                "}"
                "#threadinfo th {"
                    "border: none;"
                    "background-color: #374F7F;"
                    "color: #FFFFFF;"
                    "padding: 1px 4px 1px 4px;"
                "}"
                "#threadinfo td {"
                    "padding: 1px 4px 1px 4px;"
                "}"
                "#threadinfo tr {"
                    "background-color: #FFFFFF;"
                "}"
                "#threadinfo tr:nth-child(odd) {"
                    "background-color: #F0F0F0;"
                "}"
                "#colgroupTaskName {"
                    "width: 50%;"
                "}"
                "#colgroupCpuLoad {"
                    "width: 50%;"
                "}"
            "</style>"
            "<meta charset=\"utf-8\"/>"
            "<title>EtherNet/IP&trade; Adapter Web Server Example</title>"
            "<script src=\"main.js\"></script>"
        "</head>"
        "<body onload=\"getCpuLoad()\">"
            "<div id=\"titlearea\">"
                "<table cellspacing=\"0\" cellpadding=\"0\">"
                    "<tbody>"
                        "<tr style=\"height: 40px;\">"
                            "<td id =\"projectlogo\">"
                                "<svg x=\"0px\" y=\"0px\" width=\"186px\" height=\"36px\" viewBox=\"0 0 280 24\" >"
                                    "<path d=\"M55.485,10.266v13.53c0,1.433-0.73,1.657-1.435,1.882v0.479h6.246v-0.479c-0.702-0.227-1.434-0.445-1.434-1.882v-13.53  c0-0.815,0.253-1.014,1.323-1.014c2.925,0,4.359,0.058,5.399,1.682l0.365-0.223l-1.18-3.467H49.55l-1.181,3.467l0.366,0.223  c1.266-1.603,2.25-1.682,5.428-1.682C55.231,9.254,55.485,9.451,55.485,10.266z M62.601,12.206c0.927,0.253,1.407,0.478,1.407,1.883  v9.706c0,1.433-0.477,1.657-1.407,1.882v0.48h11.785l1.349-3.384l-0.368-0.252c-1.181,1.556-2.419,1.912-4.049,1.912h-3.039  c-0.955,0-1.18-0.021-1.18-0.958v-4.048h3.683c1.434,0,1.658,0.341,1.886,1.184h0.56v-4.048h-0.56  c-0.228,0.845-0.452,1.212-1.886,1.212H67.1v-3.404c0-0.704,0.225-0.935,1.18-0.935c0,0,2.364,0,2.588,0  c1.658,0,2.532,0.427,3.797,1.77l0.365-0.25l-1.18-3.235H62.601V12.206z M77.758,13.867l3.602,5.086l-3.406,4.838  c-1.039,1.46-1.657,1.655-2.616,1.882v0.478h5.458v-0.478c-1.041-0.197-1.491-0.593-0.788-1.685l2.384-3.72l2.51,3.616  c0.928,1.323-0.058,1.683-0.903,1.799v0.48h9.871v-0.48c-0.984-0.168-1.434-0.507-0.929-1.854l0.643-1.682h5.909l0.62,1.682  c0.395,1.1,0.253,1.686-0.901,1.854v0.48h6.076v-0.48c-0.703-0.226-1.35-0.809-1.658-1.632L98.933,11.73h-3.206l-4.5,11.481  c-0.479,1.184-0.914,1.754-1.393,2.077c-0.37-0.234-0.712-0.602-1.167-1.231l-3.955-5.617l3.197-4.516  c0.844-1.211,1.549-1.462,2.392-1.718v-0.48h-5.149v0.48c1.067,0.084,1.378,0.677,0.759,1.599l-2.23,3.303l-2.015-3.025  c-0.844-1.237-0.227-1.744,0.703-1.89v-0.477h-6.833v0.477C76.491,12.459,76.971,12.74,77.758,13.867z M96.6,14.23l2.311,6.301H94.2  L96.6,14.23z M106.614,26.157c0-0.229,0.225-0.707,0.504-0.707c0.644,0.312,1.745,0.956,3.768,0.956  c3.405,0,5.793-1.685,5.793-4.554c0-2.7-1.491-3.799-4.36-4.444l-2.616-0.592c-1.49-0.337-1.968-0.768-1.968-1.686  c0-1.069,0.842-1.772,2.7-1.772c2.079,0,3.712,0.986,4.893,2.346l0.367-0.253l-1.068-3.716h-0.676c0,0.311-0.139,0.647-0.422,0.647  c-0.621-0.338-1.576-0.87-3.206-0.87c-2.925,0-5.202,1.516-5.202,4.273c0,2.104,1.266,3.426,4.022,4.047l2.475,0.562  c1.658,0.357,2.249,0.954,2.249,1.997c0,1.433-1.151,2.193-3.037,2.193c-2.644,0-4.33-1.387-5.709-2.981l-0.368,0.253l1.154,4.304  L106.614,26.157L106.614,26.157z\"/>"
                                    "<path d=\"M125.79,7.736c0.873,0.252,1.574,0.478,1.574,1.884v14.181c0,1.433-0.702,1.659-1.574,1.882v0.478h6.526v-0.478  c-0.872-0.225-1.576-0.446-1.576-1.882V9.62c0-1.41,0.704-1.634,1.576-1.884V7.257h-6.526V7.736L125.79,7.736z M143.705,12.208  c0.843,0.252,1.323,0.478,1.323,1.884V20h-0.084l-7.256-8.272h-4.245v0.48c0.844,0.358,1.407,0.644,1.407,1.883v9.705  c0,1.349-0.478,1.66-1.407,1.882v0.48h4.809v-0.48c-0.76-0.225-1.49-0.532-1.49-1.882v-8.381h0.084l9.591,10.941h0.504V14.091  c0-1.407,0.705-1.633,1.268-1.883v-0.481h-4.5L143.705,12.208L143.705,12.208z M156.278,17.412l-2.616-0.59  c-1.491-0.338-1.97-0.769-1.97-1.688c0-1.064,0.845-1.774,2.698-1.774c2.083,0,3.716,0.987,4.896,2.346l0.367-0.253l-1.067-3.716  h-0.677c0,0.311-0.14,0.648-0.421,0.648c-0.62-0.338-1.574-0.87-3.205-0.87c-2.929,0-5.212,1.516-5.212,4.273  c0,2.103,1.267,3.426,4.023,4.048l2.471,0.562c1.66,0.357,2.253,0.955,2.253,1.997c0,1.433-1.149,2.185-3.039,2.185  c-2.638,0-4.34-1.378-5.71-2.975l-0.367,0.251l1.153,4.297h0.703c0-0.228,0.226-0.701,0.506-0.701  c0.644,0.312,1.744,0.956,3.768,0.956c3.402,0,5.792-1.682,5.792-4.551C160.637,19.156,159.146,18.057,156.278,17.412z   M161.481,11.729l-1.155,3.716l0.368,0.252c1.546-2.055,1.969-2.251,4.021-2.251c0.705,0,0.928,0.057,0.928,0.928v9.424  c0,1.432-0.421,1.656-1.573,1.882v0.479h6.189V25.68c-1.127-0.227-1.577-0.438-1.577-1.882v-9.424c0-0.871,0.226-0.928,0.931-0.928  c2.056,0,2.596,0.189,4.133,2.251l0.368-0.252l-1.153-3.716H161.481z M186.538,21.884c-0.871-1.406-1.768-2.027-2.809-2.28V19.52  c2.473-0.395,3.883-1.687,3.883-3.799c0-2.802-2.249-3.996-5.876-3.996h-7.397v0.481c0.936,0.251,1.41,0.478,1.41,1.883v9.707  c0,1.433-0.478,1.656-1.41,1.882v0.479h6.045v-0.479c-0.872-0.227-1.463-0.448-1.463-1.882v-3.602h1.491  c1.744,0,1.996,0.702,3.122,3.066c0.93,1.941,2.167,3.148,4.499,3.148c1.185,0,1.661-0.229,2.138-0.479v-0.395  C188.283,25.258,187.663,23.713,186.538,21.884z M181.394,18.51h-2.477v-3.827c0-0.929,0.229-1.18,0.936-1.18h1.6  c1.744,0,2.76,0.676,2.76,2.362C184.205,17.638,183.052,18.51,181.394,18.51z M199.336,12.208c0.729,0.172,1.434,0.478,1.434,1.884  v7.116c0,2.364-0.899,3.406-3.57,3.406c-2.84,0-3.855-1.042-3.855-3.406v-7.116c0-1.406,0.533-1.716,1.41-1.884v-0.479h-5.905v0.479  c0.87,0.141,1.433,0.478,1.433,1.884v7.343c0,3.85,3.346,4.979,6.499,4.979c3.515,0,6.016-1.185,6.016-5.211v-7.115  c0-1.407,0.428-1.683,1.298-1.885v-0.477h-4.754L199.336,12.208L199.336,12.208z M214.3,21.097l-4.166-9.368h-5.172v0.479  c0.929,0.252,1.407,0.478,1.407,1.884v9.705c0,1.263-0.535,1.598-1.407,1.882v0.48h4.585v-0.48c-0.956-0.25-1.377-0.619-1.377-1.882  v-9.056l5.201,11.644h0.396l5.062-11.644v9.056c0,1.433-0.618,1.682-1.32,1.882v0.48h5.904v-0.48  c-0.927-0.226-1.406-0.446-1.406-1.882v-9.705c0-1.406,0.479-1.633,1.406-1.884v-0.479h-5.089L214.3,21.097z M233.257,24.443h-3.038  c-0.956,0-1.181-0.021-1.181-0.954v-4.048h3.686c1.432,0,1.66,0.335,1.883,1.18h0.562v-4.047h-0.562  c-0.223,0.843-0.448,1.21-1.883,1.21h-3.686v-3.403c0-0.703,0.225-0.931,1.181-0.931c0,0,2.362,0,2.588,0  c1.658,0,2.529,0.424,3.799,1.766l0.363-0.253l-1.186-3.235h-11.25v0.481c0.932,0.251,1.408,0.478,1.408,1.883v9.707  c0,1.433-0.477,1.658-1.408,1.881v0.48h11.785l1.351-3.384l-0.368-0.252C236.124,24.079,234.886,24.443,233.257,24.443z   M248.275,12.208c0.842,0.252,1.324,0.478,1.324,1.884V20h-0.089l-7.254-8.272h-4.249v0.48c0.846,0.358,1.409,0.644,1.409,1.883  v9.705c0,1.349-0.48,1.66-1.409,1.882v0.48h4.81v-0.48c-0.77-0.225-1.49-0.532-1.49-1.882v-8.381h0.083L251,26.356h0.512V14.091  c0-1.407,0.703-1.633,1.264-1.883v-0.481h-4.498L248.275,12.208L248.275,12.208z M254.096,11.729l-1.152,3.716l0.366,0.252  c1.547-2.055,1.97-2.251,4.022-2.251c0.703,0,0.929,0.057,0.929,0.928v9.424c0,1.432-0.421,1.656-1.573,1.882v0.479h6.19V25.68  c-1.128-0.227-1.577-0.438-1.577-1.882v-9.424c0-0.871,0.229-0.928,0.936-0.928c2.054,0,2.593,0.189,4.131,2.251l0.366-0.252  l-1.152-3.716H254.096z M274.684,17.412l-2.614-0.59c-1.489-0.338-1.966-0.769-1.966-1.688c0-1.064,0.841-1.774,2.695-1.774  c2.08,0,3.713,0.987,4.897,2.346l0.354-0.253l-1.059-3.716h-0.684c0,0.311-0.146,0.648-0.423,0.648  c-0.619-0.338-1.575-0.87-3.212-0.87c-2.93,0-5.213,1.516-5.213,4.273c0,2.103,1.267,3.426,4.021,4.048l2.471,0.562  c1.647,0.357,2.253,0.955,2.253,1.997c0,1.433-1.147,2.185-3.041,2.185c-2.639,0-4.339-1.378-5.71-2.975l-0.366,0.251l1.142,4.297  h0.707c0-0.228,0.228-0.701,0.509-0.701c0.645,0.312,1.75,0.956,3.763,0.956c3.389,0,5.791-1.682,5.791-4.551  C279.044,19.156,277.552,18.057,274.684,17.412z\"/>"
                                    "<path fill=\"#CC0000\" d=\"M34.616,11.575V7.276h-6.904l-0.871,4.339h2.483l-0.795,3.999h-2.484l-0.86,4.328  c-0.067,0.294-0.117,0.506-0.147,0.71c-0.204,1.328,0.374,1.261,2.233,1.261l-0.85,4.257h-3.25c-3.072,0-5.571-0.026-4.922-3.177  c0.103-0.481,0.226-0.967,0.313-1.433l1.161-5.942h-2.389l0.797-3.999h2.388l0.869-4.339h-2.233V0.721h-8.266v13.322H1  C1.245,17.497,4.301,18,5.416,20.454c0.768,1.691,1.439,4.672,3.681,4.692c1.717,0.021,1.717-2.222,3.928-2.222  c2.208,0,3.188,4.195,5.152,7.649c1.472,2.712,3.917,4.532,6.133,4.688c1.819,0.126,2.699-0.492,2.699-0.492  c-0.491-0.988-0.737-1.973-0.737-3.455c0-2.221,1.719-4.44,3.682-5.919c2.699-1.972,5.644-2.719,6.625-2.959v-6.912  C36.579,15.524,34.616,14.474,34.616,11.575z M23.355,21.902L26.259,7.27h-3.346l-2.9,14.632H23.355z M27.156,2.644h-3.347  l-0.64,3.227h3.345L27.156,2.644z\"/>"
                                "</svg>"
                            "</td>"
                            "<td id=\"projectalign\">"
                                "<div id=\"projectname\">EtherNet/IP&trade; Adapter Web Server Example</div>"
                            "</td>"
                        "</tr>"
                    "</tbody>"
                "</table>"
            "</div>"
            "<div id=\"nav\"></div>"
            "<div id=\"content\">"
                "<table id=\"threadinfo\" width=\"50%\" >"
                    "<colgroup>"
                        "<col id=\"colgroupTaskName\">"
                    "</colgroup>"
                    "<colgroup>"
                        "<col id=\"colgroupCpuLoad\">"
                    "</colgroup>"
                    "<caption>CPU Load Summary</caption>"
                    "<tbody>"
                        "<tr>"
                            "<th>Task Name</th>"
                            "<th>CPU Load</th>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r0c0\"></th>"
                            "<td id = \"r0c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r1c0\"></th>"
                            "<td id = \"r1c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r2c0\"></th>"
                            "<td id = \"r2c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r3c0\"></th>"
                            "<td id = \"r3c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r4c0\"></th>"
                            "<td id = \"r4c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r5c0\"></th>"
                            "<td id = \"r5c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r6c0\"></th>"
                            "<td id = \"r6c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r7c0\"></th>"
                            "<td id = \"r7c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r8c0\"></th>"
                            "<td id = \"r8c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r9c0\"></th>"
                            "<td id = \"r9c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r10c0\"></th>"
                            "<td id = \"r10c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r11c0\"></th>"
                            "<td id = \"r11c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r12c0\"></th>"
                            "<td id = \"r12c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r13c0\"></th>"
                            "<td id = \"r13c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r14c0\"></th>"
                            "<td id = \"r14c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r15c0\"></th>"
                            "<td id = \"r15c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r16c0\"></th>"
                            "<td id = \"r16c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r17c0\"></th>"
                            "<td id = \"r17c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r18c0\"></th>"
                            "<td id = \"r18c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r19c0\"></th>"
                            "<td id = \"r19c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r20c0\"></th>"
                            "<td id = \"r20c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r21c0\"></th>"
                            "<td id = \"r21c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r22c0\"></th>"
                            "<td id = \"r22c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r23c0\"></th>"
                            "<td id = \"r23c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r24c0\"></th>"
                            "<td id = \"r24c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r25c0\"></th>"
                            "<td id = \"r25c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r26c0\"></th>"
                            "<td id = \"r26c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r27c0\"></th>"
                            "<td id = \"r27c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r28c0\"></th>"
                            "<td id = \"r28c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r29c0\"></th>"
                            "<td id = \"r29c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r30c0\"></th>"
                            "<td id = \"r30c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r31c0\"></th>"
                            "<td id = \"r31c1\">0 %</td>"
                        "</tr>"
                        "<tr>"
                            "<th id = \"r32c0\"></th>"
                            "<td id = \"r32c1\">0 %</td>"
                        "</tr>"
                    "</tbody>"
                "</table>"
               "</div>"
        "</body>"
    "</html>";

    static const char main_js[] =
    "function getCpuLoad()"
    "{"
        "var cpuLoad = 0;"
        "function getCpuLoadComplete()"
        "{"
            "if(cpuLoad.readyState == 4)"
            "{"
                "if(cpuLoad.status == 200)"
                "{"
                    "if(cpuLoad.responseText != null)"
                    "{"
                        "var rowsNum = 20;"
                        "var id = \"\";"
                        "var row = 0;"
                        "var data = cpuLoad.responseText;"
                        "const dataArray = data.split(\",\");"
                        "for (let i = 0; i < dataArray.length; i = i + 2)"
                        "{"
                            "row = i/2;"
                            "if (rowsNum > i/2)"
                            "{"
                                "id = \"r\" + row + \"c0\";"
                                "document.getElementById(id).innerHTML = dataArray[i];"
                                "id = \"r\" + row + \"c1\";"
                                "document.getElementById(id).innerHTML = dataArray[i + 1];"
                            "}"
                        "}"
                    "}"
                "}"
            "}"
        "}"
        "cpuLoad = new XMLHttpRequest();"
        "if(cpuLoad)"
        "{"
            "cpuLoad.open(\"GET\", \"/cpuLoad?id=\" + Math.random(), true);"
            "cpuLoad.responseType = \"text\";"
            "cpuLoad.onreadystatechange = getCpuLoadComplete;"
            "cpuLoad.send(null);"
        "}"
        "setTimeout('getCpuLoad()', 500);"
    "}";

    static const char favicon_ico[] =
    {
        0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x10, 0x10,
        0x10, 0x00, 0x01, 0x00, 0x04, 0x00, 0x28, 0x01,
        0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x10, 0x10,
        0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x68, 0x05,
        0x00, 0x00, 0x4e, 0x01, 0x00, 0x00, 0x28, 0x00,
        0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x20, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80, 0x00,
        0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80,
        0x00, 0x00, 0xc0, 0xc0, 0xc0, 0x00, 0x80, 0x80,
        0x80, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff,
        0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00,
        0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0xff,
        0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x07, 0x97, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x79, 0x97, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x07, 0x99, 0x97, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x79, 0x99, 0x99, 0x70, 0x00, 0x00, 0x07,
        0x77, 0x99, 0x97, 0x79, 0x97, 0x00, 0x00, 0x09,
        0x99, 0x99, 0x7f, 0xf7, 0x99, 0x70, 0x00, 0x79,
        0x99, 0x99, 0xf9, 0x97, 0x99, 0x97, 0x07, 0x99,
        0x99, 0x99, 0xf9, 0x9f, 0x99, 0x99, 0x79, 0x99,
        0x99, 0x99, 0xf9, 0x9f, 0x99, 0x99, 0x99, 0x99,
        0x99, 0x97, 0xf9, 0x9f, 0x79, 0x97, 0x00, 0x00,
        0x99, 0x97, 0xf7, 0x9f, 0x79, 0x90, 0x00, 0x00,
        0x99, 0x99, 0x7f, 0xf7, 0x99, 0x70, 0x00, 0x00,
        0x99, 0x99, 0x9f, 0x99, 0x00, 0x00, 0x00, 0x00,
        0x99, 0x99, 0x90, 0x99, 0x00, 0x00, 0x00, 0x00,
        0x99, 0x99, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x99, 0x99, 0x00, 0x00, 0x00, 0x00, 0xff, 0x8f,
        0xfc, 0xfd, 0xff, 0x0f, 0xfc, 0xfd, 0xfe, 0x0f,
        0xfc, 0xfd, 0xfc, 0x07, 0xfc, 0xfd, 0xe0, 0x03,
        0xfc, 0xfd, 0xe0, 0x01, 0xfc, 0xfd, 0xc0, 0x00,
        0xfc, 0xfd, 0x80, 0x00, 0xfc, 0xfd, 0x00, 0x00,
        0xfc, 0xfd, 0x00, 0x00, 0xfc, 0xfd, 0xf0, 0x01,
        0xfc, 0xfd, 0xf0, 0x01, 0xfc, 0xfd, 0xf0, 0x0f,
        0xfc, 0xfd, 0xf0, 0x4f, 0xfc, 0xfd, 0xf0, 0x7f,
        0xfc, 0xfd, 0xf0, 0xff, 0xfc, 0xfd, 0x28, 0x00,
        0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x20, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x80, 0x00,
        0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x80,
        0x00, 0x00, 0xc0, 0xc0, 0xc0, 0x00, 0xc0, 0xdc,
        0xc0, 0x00, 0xf0, 0xca, 0xa6, 0x00, 0xd4, 0xf0,
        0xff, 0x00, 0xb1, 0xe2, 0xff, 0x00, 0x8e, 0xd4,
        0xff, 0x00, 0x6b, 0xc6, 0xff, 0x00, 0x48, 0xb8,
        0xff, 0x00, 0x25, 0xaa, 0xff, 0x00, 0x00, 0xaa,
        0xff, 0x00, 0x00, 0x92, 0xdc, 0x00, 0x00, 0x7a,
        0xb9, 0x00, 0x00, 0x62, 0x96, 0x00, 0x00, 0x4a,
        0x73, 0x00, 0x00, 0x32, 0x50, 0x00, 0xd4, 0xe3,
        0xff, 0x00, 0xb1, 0xc7, 0xff, 0x00, 0x8e, 0xab,
        0xff, 0x00, 0x6b, 0x8f, 0xff, 0x00, 0x48, 0x73,
        0xff, 0x00, 0x25, 0x57, 0xff, 0x00, 0x00, 0x55,
        0xff, 0x00, 0x00, 0x49, 0xdc, 0x00, 0x00, 0x3d,
        0xb9, 0x00, 0x00, 0x31, 0x96, 0x00, 0x00, 0x25,
        0x73, 0x00, 0x00, 0x19, 0x50, 0x00, 0xd4, 0xd4,
        0xff, 0x00, 0xb1, 0xb1, 0xff, 0x00, 0x8e, 0x8e,
        0xff, 0x00, 0x6b, 0x6b, 0xff, 0x00, 0x48, 0x48,
        0xff, 0x00, 0x25, 0x25, 0xff, 0x00, 0x00, 0x00,
        0xff, 0x00, 0x00, 0x00, 0xdc, 0x00, 0x00, 0x00,
        0xb9, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x00,
        0x73, 0x00, 0x00, 0x00, 0x50, 0x00, 0xe3, 0xd4,
        0xff, 0x00, 0xc7, 0xb1, 0xff, 0x00, 0xab, 0x8e,
        0xff, 0x00, 0x8f, 0x6b, 0xff, 0x00, 0x73, 0x48,
        0xff, 0x00, 0x57, 0x25, 0xff, 0x00, 0x55, 0x00,
        0xff, 0x00, 0x49, 0x00, 0xdc, 0x00, 0x3d, 0x00,
        0xb9, 0x00, 0x31, 0x00, 0x96, 0x00, 0x25, 0x00,
        0x73, 0x00, 0x19, 0x00, 0x50, 0x00, 0xf0, 0xd4,
        0xff, 0x00, 0xe2, 0xb1, 0xff, 0x00, 0xd4, 0x8e,
        0xff, 0x00, 0xc6, 0x6b, 0xff, 0x00, 0xb8, 0x48,
        0xff, 0x00, 0xaa, 0x25, 0xff, 0x00, 0xaa, 0x00,
        0xff, 0x00, 0x92, 0x00, 0xdc, 0x00, 0x7a, 0x00,
        0xb9, 0x00, 0x62, 0x00, 0x96, 0x00, 0x4a, 0x00,
        0x73, 0x00, 0x32, 0x00, 0x50, 0x00, 0xff, 0xd4,
        0xff, 0x00, 0xff, 0xb1, 0xff, 0x00, 0xff, 0x8e,
        0xff, 0x00, 0xff, 0x6b, 0xff, 0x00, 0xff, 0x48,
        0xff, 0x00, 0xff, 0x25, 0xff, 0x00, 0xff, 0x00,
        0xff, 0x00, 0xdc, 0x00, 0xdc, 0x00, 0xb9, 0x00,
        0xb9, 0x00, 0x96, 0x00, 0x96, 0x00, 0x73, 0x00,
        0x73, 0x00, 0x50, 0x00, 0x50, 0x00, 0xff, 0xd4,
        0xf0, 0x00, 0xff, 0xb1, 0xe2, 0x00, 0xff, 0x8e,
        0xd4, 0x00, 0xff, 0x6b, 0xc6, 0x00, 0xff, 0x48,
        0xb8, 0x00, 0xff, 0x25, 0xaa, 0x00, 0xff, 0x00,
        0xaa, 0x00, 0xdc, 0x00, 0x92, 0x00, 0xb9, 0x00,
        0x7a, 0x00, 0x96, 0x00, 0x62, 0x00, 0x73, 0x00,
        0x4a, 0x00, 0x50, 0x00, 0x32, 0x00, 0xff, 0xd4,
        0xe3, 0x00, 0xff, 0xb1, 0xc7, 0x00, 0xff, 0x8e,
        0xab, 0x00, 0xff, 0x6b, 0x8f, 0x00, 0xff, 0x48,
        0x73, 0x00, 0xff, 0x25, 0x57, 0x00, 0xff, 0x00,
        0x55, 0x00, 0xdc, 0x00, 0x49, 0x00, 0xb9, 0x00,
        0x3d, 0x00, 0x96, 0x00, 0x31, 0x00, 0x73, 0x00,
        0x25, 0x00, 0x50, 0x00, 0x19, 0x00, 0xff, 0xd4,
        0xd4, 0x00, 0xff, 0xb1, 0xb1, 0x00, 0xff, 0x8e,
        0x8e, 0x00, 0xff, 0x6b, 0x6b, 0x00, 0xff, 0x48,
        0x48, 0x00, 0xff, 0x25, 0x25, 0x00, 0xff, 0x00,
        0x00, 0x00, 0xdc, 0x00, 0x00, 0x00, 0xb9, 0x00,
        0x00, 0x00, 0x96, 0x00, 0x00, 0x00, 0x73, 0x00,
        0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0xff, 0xe3,
        0xd4, 0x00, 0xff, 0xc7, 0xb1, 0x00, 0xff, 0xab,
        0x8e, 0x00, 0xff, 0x8f, 0x6b, 0x00, 0xff, 0x73,
        0x48, 0x00, 0xff, 0x57, 0x25, 0x00, 0xff, 0x55,
        0x00, 0x00, 0xdc, 0x49, 0x00, 0x00, 0xb9, 0x3d,
        0x00, 0x00, 0x96, 0x31, 0x00, 0x00, 0x73, 0x25,
        0x00, 0x00, 0x50, 0x19, 0x00, 0x00, 0xff, 0xf0,
        0xd4, 0x00, 0xff, 0xe2, 0xb1, 0x00, 0xff, 0xd4,
        0x8e, 0x00, 0xff, 0xc6, 0x6b, 0x00, 0xff, 0xb8,
        0x48, 0x00, 0xff, 0xaa, 0x25, 0x00, 0xff, 0xaa,
        0x00, 0x00, 0xdc, 0x92, 0x00, 0x00, 0xb9, 0x7a,
        0x00, 0x00, 0x96, 0x62, 0x00, 0x00, 0x73, 0x4a,
        0x00, 0x00, 0x50, 0x32, 0x00, 0x00, 0xff, 0xff,
        0xd4, 0x00, 0xff, 0xff, 0xb1, 0x00, 0xff, 0xff,
        0x8e, 0x00, 0xff, 0xff, 0x6b, 0x00, 0xff, 0xff,
        0x48, 0x00, 0xff, 0xff, 0x25, 0x00, 0xff, 0xff,
        0x00, 0x00, 0xdc, 0xdc, 0x00, 0x00, 0xb9, 0xb9,
        0x00, 0x00, 0x96, 0x96, 0x00, 0x00, 0x73, 0x73,
        0x00, 0x00, 0x50, 0x50, 0x00, 0x00, 0xf0, 0xff,
        0xd4, 0x00, 0xe2, 0xff, 0xb1, 0x00, 0xd4, 0xff,
        0x8e, 0x00, 0xc6, 0xff, 0x6b, 0x00, 0xb8, 0xff,
        0x48, 0x00, 0xaa, 0xff, 0x25, 0x00, 0xaa, 0xff,
        0x00, 0x00, 0x92, 0xdc, 0x00, 0x00, 0x7a, 0xb9,
        0x00, 0x00, 0x62, 0x96, 0x00, 0x00, 0x4a, 0x73,
        0x00, 0x00, 0x32, 0x50, 0x00, 0x00, 0xe3, 0xff,
        0xd4, 0x00, 0xc7, 0xff, 0xb1, 0x00, 0xab, 0xff,
        0x8e, 0x00, 0x8f, 0xff, 0x6b, 0x00, 0x73, 0xff,
        0x48, 0x00, 0x57, 0xff, 0x25, 0x00, 0x55, 0xff,
        0x00, 0x00, 0x49, 0xdc, 0x00, 0x00, 0x3d, 0xb9,
        0x00, 0x00, 0x31, 0x96, 0x00, 0x00, 0x25, 0x73,
        0x00, 0x00, 0x19, 0x50, 0x00, 0x00, 0xd4, 0xff,
        0xd4, 0x00, 0xb1, 0xff, 0xb1, 0x00, 0x8e, 0xff,
        0x8e, 0x00, 0x6b, 0xff, 0x6b, 0x00, 0x48, 0xff,
        0x48, 0x00, 0x25, 0xff, 0x25, 0x00, 0x00, 0xff,
        0x00, 0x00, 0x00, 0xdc, 0x00, 0x00, 0x00, 0xb9,
        0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x00, 0x73,
        0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0xd4, 0xff,
        0xe3, 0x00, 0xb1, 0xff, 0xc7, 0x00, 0x8e, 0xff,
        0xab, 0x00, 0x6b, 0xff, 0x8f, 0x00, 0x48, 0xff,
        0x73, 0x00, 0x25, 0xff, 0x57, 0x00, 0x00, 0xff,
        0x55, 0x00, 0x00, 0xdc, 0x49, 0x00, 0x00, 0xb9,
        0x3d, 0x00, 0x00, 0x96, 0x31, 0x00, 0x00, 0x73,
        0x25, 0x00, 0x00, 0x50, 0x19, 0x00, 0xd4, 0xff,
        0xf0, 0x00, 0xb1, 0xff, 0xe2, 0x00, 0x8e, 0xff,
        0xd4, 0x00, 0x6b, 0xff, 0xc6, 0x00, 0x48, 0xff,
        0xb8, 0x00, 0x25, 0xff, 0xaa, 0x00, 0x00, 0xff,
        0xaa, 0x00, 0x00, 0xdc, 0x92, 0x00, 0x00, 0xb9,
        0x7a, 0x00, 0x00, 0x96, 0x62, 0x00, 0x00, 0x73,
        0x4a, 0x00, 0x00, 0x50, 0x32, 0x00, 0xd4, 0xff,
        0xff, 0x00, 0xb1, 0xff, 0xff, 0x00, 0x8e, 0xff,
        0xff, 0x00, 0x6b, 0xff, 0xff, 0x00, 0x48, 0xff,
        0xff, 0x00, 0x25, 0xff, 0xff, 0x00, 0x00, 0xff,
        0xff, 0x00, 0x00, 0xdc, 0xdc, 0x00, 0x00, 0xb9,
        0xb9, 0x00, 0x00, 0x96, 0x96, 0x00, 0x00, 0x73,
        0x73, 0x00, 0x00, 0x50, 0x50, 0x00, 0xf2, 0xf2,
        0xf2, 0x00, 0xe6, 0xe6, 0xe6, 0x00, 0xda, 0xda,
        0xda, 0x00, 0xce, 0xce, 0xce, 0x00, 0xc2, 0xc2,
        0xc2, 0x00, 0xb6, 0xb6, 0xb6, 0x00, 0xaa, 0xaa,
        0xaa, 0x00, 0x9e, 0x9e, 0x9e, 0x00, 0x92, 0x92,
        0x92, 0x00, 0x86, 0x86, 0x86, 0x00, 0x7a, 0x7a,
        0x7a, 0x00, 0x6e, 0x6e, 0x6e, 0x00, 0x62, 0x62,
        0x62, 0x00, 0x56, 0x56, 0x56, 0x00, 0x4a, 0x4a,
        0x4a, 0x00, 0x3e, 0x3e, 0x3e, 0x00, 0x32, 0x32,
        0x32, 0x00, 0x26, 0x26, 0x26, 0x00, 0x1a, 0x1a,
        0x1a, 0x00, 0x0e, 0x0e, 0x0e, 0x00, 0xf0, 0xfb,
        0xff, 0x00, 0xa4, 0xa0, 0xa0, 0x00, 0x80, 0x80,
        0x80, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff,
        0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff, 0x00,
        0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0xff,
        0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25,
        0x28, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x28,
        0x28, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x28, 0x28,
        0x28, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x28, 0x28,
        0x28, 0x28, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x25, 0x25, 0x25, 0x25, 0x28, 0x28, 0xf6,
        0xf6, 0x23, 0x28, 0x25, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x28, 0x28, 0x28, 0x28, 0x28, 0x23, 0x23,
        0x23, 0xf6, 0x23, 0x28, 0x25, 0x00, 0x00, 0x00,
        0x25, 0x28, 0x28, 0x28, 0x28, 0x28, 0x23, 0x28,
        0x28, 0xf6, 0x28, 0x28, 0x28, 0x25, 0x00, 0x25,
        0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x23, 0x28,
        0x28, 0xf6, 0x28, 0x28, 0x28, 0x28, 0x25, 0x28,
        0x28, 0x28, 0x28, 0x28, 0x28, 0xf6, 0xf6, 0x28,
        0x28, 0xf6, 0xf6, 0x28, 0x28, 0x28, 0x28, 0x28,
        0x28, 0x28, 0x28, 0x28, 0x28, 0xf6, 0xf6, 0x28,
        0x28, 0x23, 0xf6, 0x28, 0x28, 0x25, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x23, 0x23,
        0x28, 0x23, 0x28, 0x28, 0x28, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x28, 0x23,
        0x23, 0x23, 0x28, 0x28, 0x28, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x28, 0x23,
        0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x25, 0x00,
        0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x28, 0x28, 0x28, 0x28, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x8f,
        0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0xfe, 0x0f,
        0x00, 0x00, 0xfe, 0x07, 0x00, 0x00, 0xe0, 0x03,
        0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0xc0, 0x00,
        0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x01,
        0x00, 0x00, 0xf0, 0x01, 0x00, 0x00, 0xf0, 0x0f,
        0x00, 0x00, 0xf0, 0x4f, 0x00, 0x00, 0xf0, 0xff,
        0x00, 0x00, 0xf0, 0xff, 0x00, 0x00,
    };

#ifdef  __cplusplus 
}
#endif 

#endif // APP_WEBSRV_DATA
