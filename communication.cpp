#include "communication.h"

Communication::Communication() {
    wifiSSID = nullptr;
    wifiPassword = nullptr;
    commandCallback = nullptr;
    statusCallback = nullptr;
    
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
        
        // API 엔드포인트 설정
        server.on("/move", HTTP_POST, [this]() { handleMove(); });
        server.on("/status", HTTP_GET, [this]() { handleStatus(); });
        server.on("/emergency-stop", HTTP_POST, [this]() { handleEmergencyStop(); });
        server.on("/set-speed", HTTP_POST, [this]() { handleSetSpeed(); });
        server.onNotFound([this]() { handleNotFound(); });
        
        server.begin();
        Serial.println("[Communication] API server started");
    } else {
        Serial.println();
        Serial.println("[Communication] WiFi connection failed");
    }
}

void Communication::setCommandCallback(CommandCallback callback) {
    commandCallback = callback;
    Serial.println("[Communication] Command callback set");
}

void Communication::setStatusCallback(StatusCallback callback) {
    statusCallback = callback;
    Serial.println("[Communication] Status callback set");
}

bool Communication::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String Communication::getLocalIP() {
    return WiFi.localIP().toString();
}

void Communication::handleClient() {
    server.handleClient();
}

void Communication::handleMove() {
    setCORSHeaders();
    
    if (server.hasArg("plain")) {
        String jsonCommand = server.arg("plain");
        MoveCommand command = parseCommand(jsonCommand);
        
        if (command.isValid && commandCallback != nullptr) {
            // 메인 컨트롤러에 명령 전달
            commandCallback(command);
            
            // 성공 응답
            StaticJsonDocument<200> response;
            response["success"] = true;
            response["message"] = "Command executed";
            response["command"] = commandTypeToString(command.type);
            
            String responseStr;
            serializeJson(response, responseStr);
            server.send(200, "application/json", responseStr);
            
            Serial.print("[Communication] Command executed: ");
            Serial.println(commandTypeToString(command.type));
        } else {
            // 에러 응답
            server.send(400, "application/json", 
                "{\"success\":false,\"message\":\"" + command.errorMessage + "\"}");
        }
    } else {
        server.send(400, "application/json", 
            "{\"success\":false,\"message\":\"No command data\"}");
    }
}

void Communication::handleStatus() {
    setCORSHeaders();
    
    // 상태 콜백이 설정되어 있으면 최신 상태 가져오기
    if (statusCallback != nullptr) {
        currentStatus = statusCallback();
    }
    
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
    
    if (commandCallback != nullptr) {
        MoveCommand command;
        command.type = CMD_EMERGENCY_STOP;
        command.isValid = true;
        
        commandCallback(command);
        
        server.send(200, "application/json", 
            "{\"success\":true,\"message\":\"Emergency stop activated\"}");
        
        Serial.println("[Communication] Emergency stop command sent");
    } else {
        server.send(500, "application/json", 
            "{\"success\":false,\"message\":\"Command handler not set\"}");
    }
}

void Communication::handleSetSpeed() {
    setCORSHeaders();
    
    if (server.hasArg("plain")) {
        String jsonData = server.arg("plain");
        StaticJsonDocument<200> doc;
        deserializeJson(doc, jsonData);
        
        if (doc.containsKey("speed")) {
            int speed = doc["speed"];
            if (validateSpeed(speed)) {
                if (commandCallback != nullptr) {
                    MoveCommand command;
                    command.type = CMD_SET_SPEED;
                    command.speed = speed;
                    command.isValid = true;
                    
                    commandCallback(command);
                    
                    server.send(200, "application/json", 
                        "{\"success\":true,\"message\":\"Speed updated\"}");
                    
                    Serial.print("[Communication] Speed set to: ");
                    Serial.println(speed);
                }
            } else {
                server.send(400, "application/json", 
                    "{\"success\":false,\"message\":\"Invalid speed value\"}");
            }
        } else {
            server.send(400, "application/json", 
                "{\"success\":false,\"message\":\"Speed parameter missing\"}");
        }
    } else {
        server.send(400, "application/json", 
            "{\"success\":false,\"message\":\"No data received\"}");
    }
}

void Communication::handleNotFound() {
    setCORSHeaders();
    server.send(404, "application/json", 
        "{\"success\":false,\"message\":\"Endpoint not found\"}");
}

MoveCommand Communication::parseCommand(const String& jsonCommand) {
    MoveCommand command;
    command.isValid = false;
    
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
                    
                    if (validateCoordinates(command.x, command.y) && validateSpeed(command.speed)) {
                        command.isValid = true;
                    } else {
                        command.errorMessage = "Invalid coordinates or speed";
                    }
                } else {
                    command.errorMessage = "Missing coordinates";
                }
                break;
                
            case CMD_MOVE_TO_BEACON:
                if (doc.containsKey("beaconId")) {
                    command.beaconId = doc["beaconId"].as<String>();
                    command.speed = doc.containsKey("speed") ? doc["speed"].as<int>() : 200;
                    
                    if (validateSpeed(command.speed)) {
                        command.isValid = true;
                    } else {
                        command.errorMessage = "Invalid speed";
                    }
                } else {
                    command.errorMessage = "Missing beacon ID";
                }
                break;
                
            case CMD_EMERGENCY_STOP:
                command.isValid = true;
                break;
                
            default:
                command.errorMessage = "Unknown command";
                break;
        }
    } else {
        command.errorMessage = "No command specified";
    }
    
    return command;
}

void Communication::updateStatus(const RobotStatus& status) {
    currentStatus = status;
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

String Communication::createJsonResponse(bool success, const String& message, const JsonDocument& data) {
    StaticJsonDocument<200> response;
    response["success"] = success;
    response["message"] = message;
    
    if (!data.isNull()) {
        // data를 response에 병합
        for (JsonPair kv : data.as<JsonObject>()) {
            response[kv.key()] = kv.value();
        }
    }
    
    String responseStr;
    serializeJson(response, responseStr);
    return responseStr;
}

void Communication::setCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

bool Communication::validateSpeed(int speed) {
    return speed >= 0 && speed <= 255;
}

bool Communication::validateCoordinates(double x, double y) {
    // 좌표 유효성 검사 (필요에 따라 수정)
    return !isnan(x) && !isnan(y) && isfinite(x) && isfinite(y);
}
