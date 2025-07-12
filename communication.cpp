#include "communication.h"
#include "utils.h"

Communication::Communication() : server(SERVER_PORT) {
    wifiSSID = nullptr;
    wifiPassword = nullptr;
    commandCallback = nullptr;
    statusCallback = nullptr;

    currentStatus = {0.0, 0.0, 0.0, 0.0, false, false, false, 200, 100.0, ""};
}

Communication::~Communication() {}

void Communication::begin(const char* ssid, const char* password) {
    wifiSSID = ssid;
    wifiPassword = password;

    Serial.println("[Communication] Initializing WiFi...");
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

        server.begin();
        Serial.println("[Communication] Server started on port 80");
    } else {
        Serial.println("[Communication] WiFi connection failed");
    }
}

void Communication::setCommandCallback(CommandCallback callback) {
    commandCallback = callback;
}

void Communication::setStatusCallback(StatusCallback callback) {
    statusCallback = callback;
}

bool Communication::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String Communication::getLocalIP() {
    return WiFi.localIP().toString();
}

void Communication::handleClient() {
    WiFiClient client = server.available();
    if (!client) return;

    String request = "";
    while (client.connected()) {
        if (client.available()) {
            char c = client.read();
            request += c;

            if (request.endsWith("\r\n\r\n")) break; // End of headers
        }
    }

    // Extract method and path
    String method, path;
    int firstSpace = request.indexOf(' ');
    int secondSpace = request.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > firstSpace) {
        method = request.substring(0, firstSpace);
        path = request.substring(firstSpace + 1, secondSpace);
    }

    // 요청 정보 로그 제거 - 필요시에만 출력

    // Read body (if POST)
    String body = "";
    if (method == "POST") {
        while (client.available()) {
            body += (char)client.read();
        }
    }

    handleClientRequest(client, method, path, body);

    delay(1);
    client.stop();
}

void Communication::handleClientRequest(WiFiClient& client, const String& method, const String& path, const String& body) {
    if (method == "POST" && path == "/move") {
        MoveCommand command = parseCommand(body);
        if (command.isValid && commandCallback) {
            commandCallback(command);
            JSONVar res;
            res["success"] = true;
            res["message"] = "Command executed";
            res["command"] = commandTypeToString(command.type);
            sendJsonResponse(client, 200, JSON.stringify(res));
        } else {
            JSONVar res;
            res["success"] = false;
            res["message"] = command.errorMessage;
            sendJsonResponse(client, 400, JSON.stringify(res));
        }

    } else if (method == "GET" && path == "/status") {
        if (statusCallback) {
            currentStatus = statusCallback();
        }

        JSONVar res;
        res["currentX"] = currentStatus.currentX;
        res["currentY"] = currentStatus.currentY;
        res["targetX"] = currentStatus.targetX;
        res["targetY"] = currentStatus.targetY;
        res["isMoving"] = currentStatus.isMoving;
        res["isEmergencyStop"] = currentStatus.isEmergencyStop;
        res["isMapLearning"] = currentStatus.isMapLearning;
        res["currentSpeed"] = currentStatus.currentSpeed;
        res["batteryLevel"] = currentStatus.batteryLevel;
        res["lastError"] = currentStatus.lastError;

        sendJsonResponse(client, 200, JSON.stringify(res));

    } else if (method == "POST" && path == "/emergency-stop") {
        if (commandCallback) {
            MoveCommand command;
            command.type = CMD_EMERGENCY_STOP;
            command.isValid = true;
            commandCallback(command);
            sendJsonResponse(client, 200, "{\"success\":true,\"message\":\"Emergency stop activated\"}");
        } else {
            sendJsonResponse(client, 500, "{\"success\":false,\"message\":\"Command handler not set\"}");
        }

    } else if (method == "POST" && path == "/set-speed") {
        JSONVar data = JSON.parse(body);
        if (JSON.typeof(data) == "undefined" || !data.hasOwnProperty("speed")) {
            sendJsonResponse(client, 400, "{\"success\":false,\"message\":\"Speed parameter missing\"}");
            return;
        }

        int speed = (int)data["speed"];
        if (!validateSpeed(speed)) {
            sendJsonResponse(client, 400, "{\"success\":false,\"message\":\"Invalid speed value\"}");
            return;
        }

        if (commandCallback) {
            MoveCommand command;
            command.type = CMD_SET_SPEED;
            command.speed = speed;
            command.isValid = true;
            commandCallback(command);
            sendJsonResponse(client, 200, "{\"success\":true,\"message\":\"Speed updated\"}");
        }

    } else if (method == "POST" && path == "/learn-map") {
        if (commandCallback) {
            MoveCommand command;
            command.type = CMD_LEARN_MAP;
            command.isValid = true;
            commandCallback(command);
            sendJsonResponse(client, 200, "{\"success\":true,\"message\":\"Map learning started\"}");
        } else {
            sendJsonResponse(client, 500, "{\"success\":false,\"message\":\"Command handler not set\"}");
        }

    } else if (method == "POST" && path == "/apply-learned-map") {
        if (commandCallback) {
            MoveCommand command;
            command.type = CMD_APPLY_LEARNED_MAP;
            command.isValid = true;
            commandCallback(command);
            sendJsonResponse(client, 200, "{\"success\":true,\"message\":\"Learned map applied\"}");
        } else {
            sendJsonResponse(client, 500, "{\"success\":false,\"message\":\"Command handler not set\"}");
        }

    } else if (method == "POST" && path == "/clear-map") {
        if (commandCallback) {
            MoveCommand command;
            command.type = CMD_CLEAR_MAP;
            command.isValid = true;
            commandCallback(command);
            sendJsonResponse(client, 200, "{\"success\":true,\"message\":\"Map cleared\"}");
        } else {
            sendJsonResponse(client, 500, "{\"success\":false,\"message\":\"Command handler not set\"}");
        }

    } else {
        sendJsonResponse(client, 404, "{\"success\":false,\"message\":\"Endpoint not found\"}");
    }
}

void Communication::sendJsonResponse(WiFiClient& client, int statusCode, const String& body) {
    client.println("HTTP/1.1 " + String(statusCode) + " OK");
    client.println("Content-Type: application/json");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Connection: close");
    client.println();
    client.println(body);
}

MoveCommand Communication::parseCommand(const String& jsonCommand) {
    MoveCommand command;
    command.isValid = false;

    JSONVar obj = JSON.parse(jsonCommand);
    if (JSON.typeof(obj) == "undefined") {
        command.errorMessage = "Invalid JSON";
        return command;
    }

    if (!obj.hasOwnProperty("command")) {
        command.errorMessage = "No command specified";
        return command;
    }

    String cmdStr = (const char*)obj["command"];
    command.type = stringToCommandType(cmdStr);

    switch (command.type) {
        case CMD_MOVE_TO_POSITION:
            if (obj.hasOwnProperty("x") && obj.hasOwnProperty("y")) {
                command.x = (double)obj["x"];
                command.y = (double)obj["y"];
                command.speed = obj.hasOwnProperty("speed") ? (int)obj["speed"] : 200;

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
            if (obj.hasOwnProperty("beaconId")) {
                command.beaconId = (const char*)obj["beaconId"];
                command.speed = obj.hasOwnProperty("speed") ? (int)obj["speed"] : 200;

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
    if (str == "learn_map") return CMD_LEARN_MAP;
    if (str == "apply_learned_map") return CMD_APPLY_LEARNED_MAP;
    if (str == "clear_map") return CMD_CLEAR_MAP;
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
        case CMD_LEARN_MAP: return "learn_map";
        case CMD_APPLY_LEARNED_MAP: return "apply_learned_map";
        case CMD_CLEAR_MAP: return "clear_map";
        default: return "unknown";
    }
}

bool Communication::validateSpeed(int speed) {
    return speed >= 0 && speed <= 255;
}

bool Communication::validateCoordinates(double x, double y) {
    // utils의 isValidCoordinate 함수 사용 (최대값은 100으로 설정)
    return isValidCoordinate(x, y, 100.0, 100.0);
}
