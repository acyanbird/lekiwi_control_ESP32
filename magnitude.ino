#include <WiFi.h>
#include <WebSocketsServer.h> // Include WebSockets library
#include <SCServo.h>
#include <math.h> // For cos() and sin()
#include <ArduinoJson.h> // For parsing JSON messages (install via Library Manager)

SMS_STS sms_sts;

// Define Stop constant
#define Stop 0
#define Spin_left 450

// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1 // Keeping AP mode

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// Wi-Fi AP 配置
const char *ssid = "lekiwi";

// HTTP Server on port 80 (only for serving the initial HTML)
WiFiServer server(80);

// WebSocket Server on port 81 (or another port, avoids conflict with HTTP)
WebSocketsServer webSocket = WebSocketsServer(81);

// for positive 7 backward, 8 right, 9 forward
// 舵机配置
byte ID[3] = {7, 8, 9};
byte ACC[3] = {10, 10, 10};
// Base speeds for 0.1 m/s movement
s16 Forward_Base[3] = {-1016, 0, 1016};
s16 StrafeLeft_Base[3] = {890, -1780, 890};

// Function to handle incoming WebSocket messages
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    JsonDocument doc;
    DeserializationError error;
    const char* command = nullptr;

    switch(type) {
        case WStype_TEXT:
            Serial.printf("[%u] Got Text: %s\n", num, payload);

            error = deserializeJson(doc, payload);
            if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            command = doc["command"];

            if (command) {
                Serial.print("Received command: ");
                Serial.println(command);

                if (strcmp(command, "move") == 0) {
                    // Joystick command
                    float angle_degrees = doc["angle"].as<float>();
                    float magnitude = doc["magnitude"].as<float>(); // Magnitude 0.0 to 1.0

                    Serial.print("Angle: ");
                    Serial.print(angle_degrees);
                    Serial.print(", Magnitude: ");
                    Serial.println(magnitude);

                    // Convert angle to radians
                    float angle_radians = angle_degrees * PI / 180.0;

                    // Calculate servo speeds based on angle and scaled by magnitude
                    s16 s7_calc = (s16)(magnitude * (-Forward_Base[0] * cos(angle_radians) - StrafeLeft_Base[0] * sin(angle_radians)));
                    s16 s8_calc = (s16)(magnitude * (-Forward_Base[1] * cos(angle_radians) - StrafeLeft_Base[1] * sin(angle_radians)));
                    s16 s9_calc = (s16)(magnitude * (-Forward_Base[2] * cos(angle_radians) - StrafeLeft_Base[2] * sin(angle_radians)));

                    s16 s7_scaled = (s16)(magnitude * (-1016.0 * cos(angle_radians) - 890.0 * sin(angle_radians)));
                    s16 s8_scaled = (s16)(magnitude * (1780.0 * sin(angle_radians))); // Use 1780.0 from original formula
                    s16 s9_scaled = (s16)(magnitude * (1016.0 * cos(angle_radians) - 890.0 * sin(angle_radians)));

                    Serial.print("Calculated scaled servo speeds: s7=");
                    Serial.print(s7_scaled);
                    Serial.print(", s8=");
                    Serial.print(s8_scaled);
                    Serial.print(", s9=");
                    Serial.println(s9_scaled);

                    sms_sts.WriteSpe(ID[0], s7_scaled, ACC[0]); // Servo 7
                    sms_sts.WriteSpe(ID[1], s8_scaled, ACC[1]); // Servo 8
                    sms_sts.WriteSpe(ID[2], s9_scaled, ACC[2]); // Servo 9

                } else if (strcmp(command, "stop") == 0 || strcmp(command, "end_drag") == 0) {
                     Serial.println("Action: Stopping...");
                     sms_sts.WriteSpe(ID[0], 0, ACC[0]); // Servo 7
                     sms_sts.WriteSpe(ID[1], 0, ACC[1]); // Servo 8
                     sms_sts.WriteSpe(ID[2], 0, ACC[2]); // Servo 9
                } else if (strcmp(command, "spin") == 0) {
                    int direction = doc["di"].as<int>();
                    int speed = Spin_left * direction;
                    
                    Serial.print("Action: Spinning ");
                    Serial.print(direction > 0 ? "left" : "right");
                    Serial.print(" with speed ");
                    Serial.println(speed);
                    
                    // Set each servo speed individually
                    sms_sts.WriteSpe(ID[0], speed, ACC[0]); // Servo 7
                    sms_sts.WriteSpe(ID[1], speed, ACC[1]); // Servo 8
                    sms_sts.WriteSpe(ID[2], speed, ACC[2]); // Servo 9
                }
            } else {
                Serial.println("Received JSON without 'command' field.");
            }
            break;

        case WStype_BIN:
            Serial.printf("[%u] Got Data: %u\n", num, length);
            break;

        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;

        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            // Send a welcome message or initial state
            webSocket.sendTXT(num, "Connected to Lekiwi Car Control");
        }
            break;
            
        case WStype_ERROR:
             Serial.printf("[%u] Error (%s)\n", num, payload);
             break;
    }
}


void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // Configure Wi-Fi AP
  if (!WiFi.softAP(ssid)) {
    Serial.println("Soft AP creation failed.");
    while (1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Start HTTP server (only to serve the initial page)
  server.begin();
  Serial.println("HTTP Server started on port 80");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket Server started on port 81");


  // 初始化舵机串口
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  delay(1000);

  // 设置舵机为速度模式
  // Ensure all relevant motors are in wheel mode.
  // Based on usage, ID[0](7), ID[1](8), ID[2](9) are used.
  for (int i = 0; i < 3; i++) {
    sms_sts.WheelMode(ID[i]);
    delay(10); // Small delay after mode change if necessary
  }
}

void loop() {
  // Handle WebSocket connections
  webSocket.loop();

  // Handle HTTP clients (only for initial page load)
  WiFiClient client = server.accept();

  if (client) {
    Serial.println("New HTTP Client.");
    String currentLine = "";
    // unsigned long timeout = millis();
    while (client.connected()) { // Timeout after 1s
      if (client.available()) {
        char c = client.read();

        if (c == '\n') {
            Serial.print("Received line: ");
            Serial.println(currentLine);
          // End of line or end of headers
          if (currentLine.length() == 0) { // Blank line signifies end of headers (ignoring the \r)
            // --- Send the HTML content ---
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close"); // Close after sending page
            client.println(); // Blank line after headers

            // Embed your HTML/JavaScript content here
            client.print(F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Car Control with Joystick</title>
<style>
  body {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    min-height: 100vh;
    margin: 0;
    font-family: Arial, sans-serif;
    background-color: #f0f0f0;
  }
  h1 {
    text-align: center;
    margin-bottom: 20px;
  }
  #joystick-container {
    margin-top: 20px;
    position: relative;
    width: 120px; /* Adjust size as needed */
    height: 120px; /* Adjust size as needed */
    border: 2px solid #ccc;
    border-radius: 50%;
    background-color: #fff;
    touch-action: none; /* Prevent default touch actions like scrolling */
  }
  #joystick {
    position: absolute;
    width: 50px; /* Adjust size as needed */
    height: 50px; /* Adjust size as needed */
    background-color: #007bff;
    border-radius: 50%;
    top: calc(50% - 25px); /* Center initially */
    left: calc(50% - 25px); /* Center initially */
    cursor: grab;
  }
   #controls {
       margin-top: 30px;
       text-align: center;
   }
   .spin-buttons {
       margin-bottom: 15px;
   }
   .stop-button-container {
       margin-top: 10px;
   }
   button {
       padding: 10px 20px;
       margin: 5px;
       font-size: 1em;
       cursor: pointer;
       border: none;
       border-radius: 5px;
       background-color: #007bff;
       color: white;
   }
  button:hover {
      background-color: #0056b3;
  }
  #stop-button {
      background-color: #dc3545;
  }
   #stop-button:hover {
       background-color: #c82333;
   }
   #spin-left-button, #spin-right-button {
       background-color: #28a745;
   }
   #spin-left-button:hover, #spin-right-button:hover {
       background-color: #218838;
   }
   #status {
       margin-top: 10px;
       font-size: 0.9em;
       color: #555;
   }
   .slider-container {
       margin-top: 20px;
       text-align: center;
       width: 80%;
       max-width: 300px;
   }
   .slider-container label {
       display: block;
       margin-bottom: 5px;
       color: #333;
   }
   .slider-container input[type="range"] {
       width: 100%;
       margin: 10px 0;
   }
   .slider-value {
       font-size: 0.9em;
       color: #666;
   }
</style>
</head>
<body>

<h1>Lekiwi Car Control</h1>

<div id="joystick-container">
  <div id="joystick"></div>
</div>

<div class="slider-container">
    <label for="magnitude-slider">Speed control: (0 - 2.5)</label>
    <input type="range" id="magnitude-slider" min="0" max="2.5" step="0.01" value="1">
    <div class="slider-value">Current speed: <span id="magnitude-value">0.1 m/s</span></div>
</div>

<div id="controls">
    <div class="spin-buttons">
        <button id="spin-left-button" onclick="sendSpinCommand('left')">Spin Left</button>
        <button id="spin-right-button" onclick="sendSpinCommand('right')">Spin Right</button>
    </div>
    <div class="stop-button-container">
        <button id="stop-button" onclick="sendCommand('stop')">STOP</button>
    </div>
</div>

<p id="status">Connecting...</p>

<script>
const joystickContainer = document.getElementById('joystick-container');
const joystick = document.getElementById('joystick');
const statusText = document.getElementById('status');

const containerRect = joystickContainer.getBoundingClientRect();
const containerCenterX = containerRect.width / 2;
const containerCenterY = containerRect.height / 2;
const containerRadius = containerRect.width / 2; // Assuming square container

let isDragging = false;
let ws;
let currentMagnitude = 1.0; // 默认值设为 1

// Connect to WebSocket
function connectWebSocket() {
    // Use the same IP as the HTTP server, but use port 81 for WebSocket
    // Replace location.hostname with your ESP32's AP IP if not loading from that IP
    ws = new WebSocket(`ws://${location.hostname}:81`);

    ws.onopen = function(event) {
        console.log("WebSocket connection opened");
        statusText.textContent = "Connected";
        statusText.style.color = "green";
    };

    ws.onmessage = function(event) {
        console.log("Message from server:", event.data);
        // Handle messages from ESP32 if needed
    };

    ws.onerror = function(event) {
        console.error("WebSocket error:", event);
        statusText.textContent = "WebSocket Error";
        statusText.style.color = "red";
    };

    ws.onclose = function(event) {
        console.log("WebSocket connection closed:", event.code, event.reason);
         statusText.textContent = "Disconnected. Retrying...";
         statusText.style.color = "orange";
         setTimeout(connectWebSocket, 5000); // Attempt to reconnect after 5 seconds
    };
}

// Start the WebSocket connection
connectWebSocket();

// 添加滑动条事件监听
const magnitudeSlider = document.getElementById('magnitude-slider');
const magnitudeValue = document.getElementById('magnitude-value');

magnitudeSlider.addEventListener('input', function() {
    currentMagnitude = parseFloat(this.value);
    magnitudeValue.textContent = (currentMagnitude * 0.1).toFixed(2) + ' m/s';
});

// Function to send commands via WebSocket
function sendCommand(commandType, angle = 0, magnitude = 0) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        const message = {
            command: commandType,
            angle: angle,
            magnitude: commandType === 'move' ? currentMagnitude : 0 // 移动的时候使用划条，非移动就立刻停止
        };
        ws.send(JSON.stringify(message));
        console.log("Sent:", message);
    } else {
        console.warn("WebSocket not connected.");
        statusText.textContent = "Disconnected. Cannot send command.";
        statusText.style.color = "red";
    }
}

// Function to handle spin commands
function sendSpinCommand(direction) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        const message = {
            command: "spin",
            di: direction === "left" ? 1 : -1
        };
        ws.send(JSON.stringify(message));
        console.log("Sent spin command:", message);
    } else {
        console.warn("WebSocket not connected.");
        statusText.textContent = "Disconnected. Cannot send command.";
        statusText.style.color = "red";
    }
}

// Handle touch/mouse start
function startDrag(e) {
    isDragging = true;
    joystick.style.transition = 'none'; // Remove transition during drag
    e.preventDefault(); // Prevent scrolling on touch
}

// Handle touch/mouse move
function onDrag(e) {
    if (!isDragging) return;

    const clientX = e.clientX || e.touches[0].clientX;
    const clientY = e.clientY || e.touches[0].clientY;

    const containerBoundingRect = joystickContainer.getBoundingClientRect();
    const offsetX = clientX - containerBoundingRect.left - containerCenterX;
    const offsetY = clientY - containerBoundingRect.top - containerCenterY; // Y is positive down

    // Calculate distance from center
    let distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

    // Limit distance to the container radius
    if (distance > containerRadius) {
        const angle = Math.atan2(offsetY, offsetX);
        offsetX = containerRadius * Math.cos(angle);
        offsetY = containerRadius * Math.sin(angle);
        distance = containerRadius; // Set distance to max radius
    }

    // Update joystick position
    joystick.style.left = (offsetX + containerCenterX - joystick.offsetWidth / 2) + 'px';
    joystick.style.top = (offsetY + containerCenterY - joystick.offsetHeight / 2) + 'px';

    // Calculate angle and magnitude
    // atan2(y, x) gives angle in radians from -PI to PI
    // 0 degrees usually points along positive X axis. Robot 'forward' is often Y axis.
    // Need to map this to robot's forward direction (e.g., 0 degrees = positive Y, 90 = positive X)
    // Let's map angle: 0 degrees = straight UP (forward), 90 = RIGHT (strafe right), 180 = DOWN (backward), 270 = LEFT (strafe left)
    // Math.atan2(y, x) gives angle relative to positive X.
    // For UP (forward), y is negative, x is near 0. Angle is -PI/2 (-90 deg).
    // For RIGHT (strafe right), y is near 0, x is positive. Angle is 0 deg.
    // For DOWN (backward), y is positive, x is near 0. Angle is PI/2 (90 deg).
    // For LEFT (strafe left), y is near 0, x is negative. Angle is PI or -PI (180 deg).

    // Let's recalculate angle based on UP being 0 degrees
    // We need the angle of the vector (offsetX, -offsetY) relative to the positive Y axis.
    // Angle relative to positive X is atan2(offsetY, offsetX).
    // To get angle relative to positive Y (UP), we can use atan2(offsetX, -offsetY).
    // This gives angle from -PI to PI. Convert to 0-360 degrees.

    let angle_radians = Math.atan2(offsetX, -offsetY); // Angle relative to positive Y (UP)
    let angle_degrees = angle_radians * (180 / Math.PI);

    // Convert angle_degrees from -180 to 180 range to 0 to 360 range
    if (angle_degrees < 0) {
        angle_degrees += 360;
    }

    // Send data via WebSocket
    sendCommand('move', angle_degrees, currentMagnitude); // 使用滑动条的值
}

// Handle touch/mouse end
function endDrag() {
    isDragging = false;
    joystick.style.transition = 'top 0.3s ease-out, left 0.3s ease-out'; // Add transition back
    // Snap joystick back to center
    joystick.style.top = (containerCenterY - joystick.offsetHeight / 2) + 'px';
    joystick.style.left = (containerCenterX - joystick.offsetWidth / 2) + 'px';

    // Send stop command or zero magnitude command
    // sendCommand('move', 0, 0); // Send zero magnitude when released
    sendCommand('end_drag'); // Send end drag to stop the car

}

// Add event listeners
joystickContainer.addEventListener('mousedown', startDrag);
document.addEventListener('mousemove', onDrag); // Listen on document to handle drag outside container
document.addEventListener('mouseup', endDrag);

joystickContainer.addEventListener('touchstart', startDrag);
document.addEventListener('touchmove', onDrag);
document.addEventListener('touchend', endDrag);
document.addEventListener('touchcancel', endDrag); // Handle touch interruption

// Initial positioning (redundant if CSS centers it, but good practice)
window.onload = () => {
    const initialTop = containerCenterY - joystick.offsetHeight / 2;
    const initialLeft = containerCenterX - joystick.offsetWidth / 2;
    joystick.style.top = initialTop + 'px';
    joystick.style.left = initialLeft + 'px';
};


</script>

</body>
</html>
)rawliteral"));

            break; // We've sent the page, break the while loop
          }
          currentLine = ""; // Start a new line
        } else if (c != '\r') {
          currentLine += c; // Add character to the line
        }
      }
       // Reset timeout if data is available
    //    if (client.available()) timeout = millis();
    } // End while client.connected()

    // Close the connection with the client after serving the page
    client.stop();
    Serial.println("HTTP Client Disconnected.");
  } // End if (client)
}
