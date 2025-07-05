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
};

class Communication {
public:
    Communication();
    ~Communication();
    
    // 초기화
    void begin(const char* ssid, const char* password);
    
    // WiFi 연결 상태 확인
    bool isConnected();
    
    // 웹서버 처리
    void handleClient();
    
    // 명령 처리
    MoveCommand parseCommand(const String& jsonCommand);
    String createResponse(const RobotStatus& status);
    
    // 상태 업데이트
    void updateStatus(const RobotStatus& status);
    
    // 현재 상태 반환
    RobotStatus getCurrentStatus();
    
    // 에러 메시지 설정
    void setError(const String& error);

private:
    WebServer server;
    RobotStatus currentStatus;
    
    // WiFi 설정
    const char* wifiSSID;
    const char* wifiPassword;
    
    // 웹서버 포트
    static const int SERVER_PORT = 80;
    
    // API 엔드포인트 핸들러
    void handleRoot();
    void handleMove();
    void handlePosition();
    void handleStatus();
    void handleEmergencyStop();
    void handleSetSpeed();
    void handleNotFound();
    
    // JSON 파싱 헬퍼 함수
    CommandType stringToCommandType(const String& str);
    String commandTypeToString(CommandType type);
    
    // CORS 헤더 설정
    void setCORSHeaders();
    
    // JSON 응답 생성
    String createJsonResponse(bool success, const String& message, const JsonDocument& data);
};

#endif
