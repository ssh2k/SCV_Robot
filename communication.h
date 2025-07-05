#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// 명령 타입 정의
enum CommandType {
    CMD_MOVE_TO_POSITION,
    CMD_MOVE_TO_BEACON,
    CMD_GET_POSITION,
    CMD_GET_STATUS,
    CMD_EMERGENCY_STOP,
    CMD_SET_SPEED,
    CMD_UNKNOWN
};

// 로봇 상태 구조체
struct RobotStatus {
    double currentX, currentY;
    double targetX, targetY;
    bool isMoving;
    bool isEmergencyStop;
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
    
    // 초기화
    void begin(const char* ssid, const char* password);
    
    // 콜백 설정 (메인 컨트롤러와 연결)
    void setCommandCallback(CommandCallback callback);
    void setStatusCallback(StatusCallback callback);
    
    // WiFi 관리
    bool isConnected();
    String getLocalIP();
    
    // 웹서버 처리
    void handleClient();
    
    // 명령 처리
    MoveCommand parseCommand(const String& jsonCommand);
    
    // 상태 업데이트
    void updateStatus(const RobotStatus& status);
    void setError(const String& error);

private:
    WebServer server;
    RobotStatus currentStatus;
    
    // WiFi 설정
    const char* wifiSSID;
    const char* wifiPassword;
    
    // 콜백 함수
    CommandCallback commandCallback;
    StatusCallback statusCallback;
    
    // 웹서버 포트
    static const int SERVER_PORT = 80;
    
    // API 엔드포인트 핸들러
    void handleMove();
    void handleStatus();
    void handleEmergencyStop();
    void handleSetSpeed();
    void handleNotFound();
    
    // JSON 파싱 헬퍼 함수
    CommandType stringToCommandType(const String& str);
    String commandTypeToString(CommandType type);
    
    // 응답 생성
    String createJsonResponse(bool success, const String& message, const JsonDocument& data = JsonDocument());
    
    // CORS 헤더 설정
    void setCORSHeaders();
    
    // 입력 검증
    bool validateSpeed(int speed);
    bool validateCoordinates(double x, double y);
};

#endif
