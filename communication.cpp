#include "communication.h"

Communication::Communication() {
    wifiSSID = nullptr;
    wifiPassword = nullptr;
    
    // 상태 초기화
    currentStatus.currentX = 0.0;
    currentStatus.currentY = 0.0;
    currentStatus.targetX = 0.0;
    currentStatus.targetY = 0.0;
    currentStatus.isMoving = false;
    currentStatus.isEmergencyStop = false;
    currentStatus.currentSpeed = 200;
    currentStatus.batteryLevel = 100.0;
    currentStatus.lastError = "";
}

Communication::~Communication() {
    // 웹서버 정리
    server.close();
}

void Communication::begin(const char* ssid, const char* password) {
    wifiSSID = ssid;
    wifiPassword = password;
    
    Serial.println("[Communication] Initializing WiFi...");
    
    // WiFi 연결
    WiFi.begin(wifiSSID, wifiPassword);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[Communication] WiFi connected. IP: ");
        Serial.println(WiFi.localIP());
        
        // 웹서버 설정
        server.on("/", HTTP_GET, [this]() { handleRoot(); });
        server.on("/move", HTTP_POST, [this]() { handleMove(); });
        server.on("/position", HTTP_GET, [this]() { handlePosition(); });
        server.on("/status", HTTP_GET, [this]() { handleStatus(); });
        server.on("/emergency-stop", HTTP_POST, [this]() { handleEmergencyStop(); });
        server.on("/set-speed", HTTP_POST, [this]() { handleSetSpeed(); });
        server.onNotFound([this]() { handleNotFound(); });
        
        server.begin();
        Serial.println("[Communication] Web server started");
    } else {
        Serial.println();
        Serial.println("[Communication] WiFi connection failed");
    }
}

bool Communication::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void Communication::handleClient() {
    server.handleClient();
}

void Communication::handleRoot() {
    setCORSHeaders();
    
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>SCV Robot Control</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .container { max-width: 600px; margin: 0 auto; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }
        input, button { margin: 5px; padding: 8px; }
        button { background: #007bff; color: white; border: none; border-radius: 3px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .status { background: #f8f9fa; padding: 10px; border-radius: 3px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>SCV Robot Control</h1>
        
        <div class="section">
            <h3>Move to Position</h3>
            <input type="number" id="posX" placeholder="X coordinate" step="0.1">
            <input type="number" id="posY" placeholder="Y coordinate" step="0.1">
            <input type="number" id="speed" placeholder="Speed (0-255)" min="0" max="255" value="200">
            <button onclick="moveToPosition()">Move</button>
        </div>
        
        <div class="section">
            <h3>Emergency Stop</h3>
            <button onclick="emergencyStop()" style="background: #dc3545;">Emergency Stop</button>
        </div>
        
        <div class="section">
            <h3>Status</h3>
            <div id="status" class="status">Loading...</div>
            <button onclick="updateStatus()">Refresh Status</button>
        </div>
    </div>
    
    <script>
        async function moveToPosition() {
            const x = document.getElementById('posX').value;
            const y = document.getElementById('posY').value;
            const speed = document.getElementById('speed').value;
            
            const response = await fetch('/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    command: 'move_to_position',
                    x: parseFloat(x),
                    y: parseFloat(y),
                    speed: parseInt(speed)
                })
            });
            
            const result = await response.json();
            alert(result.message);
            updateStatus();
        }
        
        async function emergencyStop() {
            const response = await fetch('/emergency-stop', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: 'emergency_stop'})
            });
            
            const result = await response.json();
            alert(result.message);
            updateStatus();
        }
        
        async function updateStatus() {
            const response = await fetch('/status');
            const status = await response.json();
            
            document.getElementById('status').innerHTML = `
                <strong>Current Position:</strong> (${status.currentX.toFixed(2)}, ${status.currentY.toFixed(2)})<br>
                <strong>Target Position:</strong> (${status.targetX.toFixed(2)}, ${status.targetY.toFixed(2)})<br>
                <strong>Moving:</strong> ${status.isMoving ? 'Yes' : 'No'}<br>
                <strong>Speed:</strong> ${status.currentSpeed}<br>
                <strong>Battery:</strong> ${status.batteryLevel.toFixed(1)}%<br>
                <strong>Error:</strong> ${status.lastError || 'None'}
            `;
        }
        
        // 초기 상태 로드
        updateStatus();
        // 5초마다 상태 업데이트
        setInterval(updateStatus, 5000);
    </script>
</body>
</html>
    )";
    
    server.send(200, "text/html", html);
}

void Communication::handleMove() {
    setCORSHeaders();
    
    if (server.hasArg("plain")) {
        String jsonCommand = server.arg("plain");
        MoveCommand command = parseCommand(jsonCommand);
        
        // 명령 처리 (메인 로직에서 처리)
        updateStatus(currentStatus);
        
        StaticJsonDocument<200> response;
        response["success"] = true;
        response["message"] = "Move command received";
        response["command"] = commandTypeToString(command.type);
        
        String responseStr;
        serializeJson(response, responseStr);
        server.send(200, "application/json", responseStr);
    } else {
        server.send(400, "application/json", "{\"success\":false,\"message\":\"No command data\"}");
    }
}

void Communication::handlePosition() {
    setCORSHeaders();
    
    StaticJsonDocument<200> response;
    response["currentX"] = currentStatus.currentX;
    response["currentY"] = currentStatus.currentY;
    response["targetX"] = currentStatus.targetX;
    response["targetY"] = currentStatus.targetY;
    
    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
}

void Communication::handleStatus() {
    setCORSHeaders();
    
    StaticJsonDocument<300> response;
    response["currentX"] = currentStatus.currentX;
    response["currentY"] = currentStatus.currentY;
    response["targetX"] = currentStatus.targetX;
    response["targetY"] = currentStatus.targetY;
    response["isMoving"] = currentStatus.isMoving;
    response["isEmergencyStop"] = currentStatus.isEmergencyStop;
    response["currentSpeed"] = currentStatus.currentSpeed;
    response["batteryLevel"] = currentStatus.batteryLevel;
    response["lastError"] = currentStatus.lastError;
    
    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
}

void Communication::handleEmergencyStop() {
    setCORSHeaders();
    
    currentStatus.isEmergencyStop = true;
    currentStatus.isMoving = false;
    
    StaticJsonDocument<200> response;
    response["success"] = true;
    response["message"] = "Emergency stop activated";
    
    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
}

void Communication::handleSetSpeed() {
    setCORSHeaders();
    
    if (server.hasArg("plain")) {
        String jsonData = server.arg("plain");
        StaticJsonDocument<200> doc;
        deserializeJson(doc, jsonData);
        
        if (doc.containsKey("speed")) {
            int speed = doc["speed"];
            if (speed >= 0 && speed <= 255) {
                currentStatus.currentSpeed = speed;
                server.send(200, "application/json", "{\"success\":true,\"message\":\"Speed updated\"}");
            } else {
                server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid speed value\"}");
            }
        } else {
            server.send(400, "application/json", "{\"success\":false,\"message\":\"Speed parameter missing\"}");
        }
    } else {
        server.send(400, "application/json", "{\"success\":false,\"message\":\"No data received\"}");
    }
}

void Communication::handleNotFound() {
    setCORSHeaders();
    server.send(404, "application/json", "{\"success\":false,\"message\":\"Endpoint not found\"}");
}

MoveCommand Communication::parseCommand(const String& jsonCommand) {
    MoveCommand command;
    command.type = CMD_UNKNOWN;
    
    StaticJsonDocument<300> doc;
    deserializeJson(doc, jsonCommand);
    
    if (doc.containsKey("command")) {
        String cmdStr = doc["command"].as<String>();
        command.type = stringToCommandType(cmdStr);
        
        switch (command.type) {
            case CMD_MOVE_TO_POSITION:
                if (doc.containsKey("x") && doc.containsKey("y")) {
                    command.x = doc["x"].as<double>();
                    command.y = doc["y"].as<double>();
                    command.speed = doc.containsKey("speed") ? doc["speed"].as<int>() : 200;
                }
                break;
                
            case CMD_MOVE_TO_BEACON:
                if (doc.containsKey("beaconId")) {
                    command.beaconId = doc["beaconId"].as<String>();
                    command.speed = doc.containsKey("speed") ? doc["speed"].as<int>() : 200;
                }
                break;
                
            case CMD_EMERGENCY_STOP:
                // 추가 파라미터 없음
                break;
                
            default:
                command.type = CMD_UNKNOWN;
                break;
        }
    }
    
    return command;
}

String Communication::createResponse(const RobotStatus& status) {
    StaticJsonDocument<300> doc;
    doc["currentX"] = status.currentX;
    doc["currentY"] = status.currentY;
    doc["targetX"] = status.targetX;
    doc["targetY"] = status.targetY;
    doc["isMoving"] = status.isMoving;
    doc["isEmergencyStop"] = status.isEmergencyStop;
    doc["currentSpeed"] = status.currentSpeed;
    doc["batteryLevel"] = status.batteryLevel;
    doc["lastError"] = status.lastError;
    
    String response;
    serializeJson(doc, response);
    return response;
}

void Communication::updateStatus(const RobotStatus& status) {
    currentStatus = status;
}

RobotStatus Communication::getCurrentStatus() {
    return currentStatus;
}

void Communication::setError(const String& error) {
    currentStatus.lastError = error;
}

CommandType Communication::stringToCommandType(const String& str) {
    if (str == "move_to_position") return CMD_MOVE_TO_POSITION;
    if (str == "move_to_beacon") return CMD_MOVE_TO_BEACON;
    if (str == "get_position") return CMD_GET_POSITION;
    if (str == "get_status") return CMD_GET_STATUS;
    if (str == "emergency_stop") return CMD_EMERGENCY_STOP;
    if (str == "set_speed") return CMD_SET_SPEED;
    return CMD_UNKNOWN;
}

String Communication::commandTypeToString(CommandType type) {
    switch (type) {
        case CMD_MOVE_TO_POSITION: return "move_to_position";
        case CMD_MOVE_TO_BEACON: return "move_to_beacon";
        case CMD_GET_POSITION: return "get_position";
        case CMD_GET_STATUS: return "get_status";
        case CMD_EMERGENCY_STOP: return "emergency_stop";
        case CMD_SET_SPEED: return "set_speed";
        default: return "unknown";
    }
}

void Communication::setCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}
