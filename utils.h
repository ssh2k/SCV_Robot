#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <cmath>

// 거리 계산 함수
double calculateDistance(double x1, double y1, double x2, double y2);

// 각도 정규화 함수 (-π ~ π 범위로)
void normalizeAngle(double& angle);

// 현재 로봇 각도 반환 (실제 센서 연동 필요)
double getCurrentRobotAngle();

// 배터리 레벨 반환 (실제 센서 연동 필요)
double getBatteryLevel();

// 속도 제한 함수
int constrainSpeed(int speed, int minSpeed, int maxSpeed);

// 좌표 유효성 검사
bool isValidCoordinate(double x, double y, double maxX, double maxY);

// 그리드 좌표 변환
int worldToGrid(double worldCoord, double cellSize);
double gridToWorld(int gridCoord, double cellSize);

// Pololu TB9051FTG 모터 드라이버 관련 유틸리티
// 모터 방향 설정 헬퍼 함수
void setMotorDirection(int in1Pin, int in2Pin, int pwmPin, int speed);

// 모터 안전 정지 함수
void safeMotorStop(int in1Pin, int in2Pin, int pwmPin);

// 모터 테스트 함수
void testMotor(int in1Pin, int in2Pin, int pwmPin, const char* motorName);

// 모터 속도 매핑 함수 (0-100%를 0-255로 변환)
int mapSpeedPercent(int percent);

// 모터 방향 검증 함수
bool isValidMotorDirection(int speed);

#endif 