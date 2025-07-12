#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <Arduino_JSON.h>

// 명령 타입 정의
enum CommandType {
    CMD_MOVE_TO_POSITION,
    CMD_MOVE_TO_BEACON,
    CMD_GET_POSITION,
    CMD_GET_STATUS,
    CMD_EMERGENCY_STOP,
    CMD_SET_SPEED,
    CMD_LEARN_MAP,           // 맵 학습 명령 추가
    CMD_APPLY_LEARNED_MAP,   // 학습된 맵 적용
    CMD_CLEAR_MAP,           // 맵 초기화
    CMD_UNKNOWN
};

// 로봇 상태 구조체
struct RobotStatus {
    double currentX, currentY;
    double targetX, targetY;
    bool isMoving;
    bool isEmergencyStop;
    bool isMapLearning;      // 맵 학습 상태 추가
    int currentSpeed;
    double batteryLevel;
    String lastError;
};

// 이동 명령 구조체
struct MoveCommand {
    CommandType type;
    double x, y;
    int speed;
    String beaconId;
    bool isValid;
    String errorMessage;
};

// 콜백 함수 타입 정의
typedef void (*CommandCallback)(const MoveCommand& command);
typedef RobotStatus (*StatusCallback)();

class Communication {
public:
    Communication();
    ~Communication();

    void begin(const char* ssid, const char* password);

    void setCommandCallback(CommandCallback callback);
    void setStatusCallback(StatusCallback callback);

    bool isConnected();
    String getLocalIP();
    void handleClient();

    MoveCommand parseCommand(const String& jsonCommand);
    void updateStatus(const RobotStatus& status);
    void setError(const String& error);

private:
    WiFiServer server;
    RobotStatus currentStatus;
    const char* wifiSSID;
    const char* wifiPassword;

    CommandCallback commandCallback;
    StatusCallback statusCallback;

    static const int SERVER_PORT = 80;

    void handleClientRequest(WiFiClient& client, const String& method, const String& path, const String& body);
    void sendJsonResponse(WiFiClient& client, int statusCode, const String& body);
    CommandType stringToCommandType(const String& str);
    String commandTypeToString(CommandType type);
    String createJsonResponse(bool success, const String& message, JSONVar data = JSONVar());
    bool validateSpeed(int speed);
    bool validateCoordinates(double x, double y);
};

#endif
