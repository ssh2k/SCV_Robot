# SCV Robot - 자율주행 이동형 선반 로봇

아두이노 WiFi R4 보드를 기반으로 한 자율주행 이동형 선반 로봇입니다. BLE 비콘을 통한 위치 인식, A* 알고리즘 기반 경로 탐색, WiFi 원격 제어, 그리고 자동 맵 학습 기능을 제공합니다.

## 🚀 주요 기능

### 🤖 로봇 제어 시스템
- **정밀 모터 제어**: PWM을 통한 좌우 바퀴 독립 제어
- **다양한 이동 모드**: 전진, 후진, 좌회전, 우회전, 곡선 이동
- **안전 기능**: 긴급 정지, 부드러운 정지, 속도 제한
- **캘리브레이션**: 모터 캘리브레이션 및 테스트 기능

### 📡 실시간 위치 인식
- **BLE 비콘 삼각측량**: 3개 비콘을 통한 정밀 위치 추적
- **RSSI 기반 거리 측정**: 실시간 신호 강도 분석
- **위치 신뢰도 평가**: 측정 정확도 자동 평가

### 🗺️ 지능형 경로 탐색
- **A* 알고리즘**: 최적 경로 탐색 및 장애물 회피
- **동적 경로 재계산**: 실시간 장애물 감지 시 경로 수정
- **경로 최적화**: 불필요한 경유점 자동 제거

### 🧠 자동 맵 학습
- **초음파 센서 기반**: 주변 환경 자동 탐지
- **장애물 매핑**: 실시간 장애물 정보 수집
- **학습된 맵 적용**: 탐지된 장애물을 경로 탐색에 반영

### 🌐 WiFi 원격 제어
- **HTTP REST API**: JSON 기반 명령 처리
- **실시간 상태 모니터링**: 로봇 상태 및 위치 정보 제공
- **콜백 패턴**: 모듈 간 느슨한 결합 구조

## 🏗️ 시스템 아키텍처

```
Next.js App (UI/UX)
    ↓ HTTP API 호출
Communication (명령 수신/파싱)
    ↓ 콜백 함수
Main Controller (로봇 제어 로직)
    ↓ 모듈 호출
MotorControl | BeaconManager | Pathfinder | MapLearner
    ↓ 유틸리티 함수
Utils (공통 함수들)
```

## 📁 프로젝트 구조

```
SCV_Robot/
├── SCV_Robot.ino          # 메인 프로그램 (로봇 제어 로직)
├── motorControl.h/cpp     # 모터 제어 모듈
├── beaconManager.h/cpp    # BLE 비콘 위치 인식 모듈
├── pathfinder.h/cpp       # A* 경로 탐색 모듈
├── mapLearner.h/cpp       # 자동 맵 학습 모듈
├── communication.h/cpp    # WiFi 통신 및 API 서버
├── utils.h/cpp            # 공통 유틸리티 함수
└── README.md              # 프로젝트 문서
```

## 🔧 하드웨어 요구사항

### 필수 구성품
- **아두이노 WiFi R4** (메인 컨트롤러)
- **DC 모터 2개** (좌우 바퀴 구동)
- **모터 드라이버** (L298N 또는 H-브리지)
- **BLE 비콘 3개** (위치 인식용)
- **초음파 센서** (장애물 감지 및 맵 학습)
- **배터리** (12V 권장)

### 선택 구성품
- **자이로스코프** (방향 센서)
- **엔코더** (정밀한 위치 추적)
- **배터리 모니터링 모듈**

## ⚙️ 설치 및 설정

### 1. Arduino IDE 설정
1. Arduino IDE 설치 (최신 버전 권장)
2. Arduino R4 보드 패키지 설치
3. 필요한 라이브러리 설치:
   ```bash
   # Arduino IDE 라이브러리 매니저에서 설치
   - WiFi (기본 포함)
   - BLEDevice (기본 포함)
   - ArduinoJson
   ```

### 2. 하드웨어 연결
```
모터 연결:
- LEFT_MOTOR_PWM_PIN: 9   (PWM)
- LEFT_MOTOR_DIR_PIN: 8   (방향)
- RIGHT_MOTOR_PWM_PIN: 10 (PWM)
- RIGHT_MOTOR_DIR_PIN: 11 (방향)

초음파 센서 연결:
- TRIG_PIN: 12
- ECHO_PIN: 13

비콘 설정 (실제 환경에 맞게 수정):
- 비콘 1: (0, 0) 위치
- 비콘 2: (10, 0) 위치  
- 비콘 3: (5, 10) 위치
```

### 3. 설정 파일 수정
`SCV_Robot.ino`에서 다음 설정을 수정하세요:

```cpp
// WiFi 설정
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// 그리드 설정
const int GRID_WIDTH = 20;
const int GRID_HEIGHT = 20;
const double GRID_CELL_SIZE = 0.5; // 미터 단위

// 시스템 설정
const unsigned long POSITION_UPDATE_INTERVAL = 1000; // 1초
const unsigned long BEACON_SCAN_INTERVAL = 5000;     // 5초
const double WAYPOINT_REACH_THRESHOLD = 0.3;        // 30cm
```

## 🚀 사용법

### 1. 로봇 실행
1. Arduino IDE에서 `SCV_Robot.ino` 업로드
2. 시리얼 모니터로 초기화 과정 확인
3. WiFi 연결 및 IP 주소 확인

### 2. API 서버 접속
로봇의 IP 주소로 HTTP API 호출:
```
http://[로봇_IP_주소]/status
```

### 3. 제어 명령
- **위치 이동**: POST `/move` - 특정 좌표로 이동
- **긴급 정지**: POST `/emergency-stop` - 즉시 정지
- **속도 조절**: POST `/set-speed` - 이동 속도 설정
- **맵 학습**: POST `/learn-map` - 자동 맵 학습 시작
- **상태 확인**: GET `/status` - 로봇 상태 조회

## 📡 API 명세

### 이동 명령
```json
POST /move
Content-Type: application/json

{
    "command": "move_to_position",
    "x": 5.0,
    "y": 3.0,
    "speed": 200
}

Response:
{
    "success": true,
    "message": "Command executed",
    "command": "move_to_position"
}
```

### 상태 조회
```json
GET /status

Response:
{
    "currentX": 2.5,
    "currentY": 1.8,
    "targetX": 5.0,
    "targetY": 3.0,
    "isMoving": true,
    "currentSpeed": 200,
    "batteryLevel": 85.5,
    "isMapLearning": false,
    "lastError": ""
}
```

### 맵 학습 명령
```json
POST /learn-map
Content-Type: application/json

{
    "command": "learn_map"
}

Response:
{
    "success": true,
    "message": "Map learning started"
}
```

### 긴급 정지
```json
POST /emergency-stop
Content-Type: application/json

{
    "command": "emergency_stop"
}

Response:
{
    "success": true,
    "message": "Emergency stop activated"
}
```

## 🔧 개발 가이드

### 모듈 구조
각 모듈은 독립적으로 설계되어 있어 유지보수와 확장이 용이합니다:

- **MotorControl**: 모터 제어 및 이동 로직
- **BeaconManager**: BLE 비콘 스캔 및 위치 계산
- **Pathfinder**: A* 알고리즘 기반 경로 탐색
- **MapLearner**: 초음파 센서를 통한 맵 학습
- **Communication**: WiFi 통신 및 API 서버
- **Utils**: 공통 유틸리티 함수들

### 새로운 기능 추가
1. 해당 모듈의 `.h` 파일에 인터페이스 정의
2. `.cpp` 파일에 구현 작성
3. `SCV_Robot.ino`에서 모듈 통합
4. 필요시 communication 모듈에 API 엔드포인트 추가

### Communication 모듈 확장 예시
```cpp
// 새로운 명령 타입 추가
enum CommandType {
    CMD_MOVE_TO_POSITION,
    CMD_EMERGENCY_STOP,
    CMD_SET_SPEED,
    CMD_LEARN_MAP,
    CMD_NEW_FEATURE,  // 새로운 기능
    CMD_UNKNOWN
};

// API 엔드포인트 추가
void Communication::handleNewFeature() {
    // 새로운 기능 처리
}

// 메인 컨트롤러에서 처리
void handleCommand(const MoveCommand& command) {
    switch(command.type) {
        case CMD_NEW_FEATURE:
            // 새로운 기능 실행
            break;
    }
}
```

## 🐛 문제 해결

### 일반적인 문제
1. **WiFi 연결 실패**: SSID/비밀번호 확인
2. **비콘 인식 안됨**: 비콘 주소 및 거리 확인
3. **모터 동작 안됨**: 핀 연결 및 전원 공급 확인
4. **경로 찾기 실패**: 장애물 설정 및 그리드 크기 확인
5. **API 응답 없음**: 콜백 함수 설정 확인
6. **맵 학습 안됨**: 초음파 센서 연결 확인

### 성능 최적화
- 비콘 스캔 주기 조정 (`BEACON_SCAN_INTERVAL`)
- 경로 탐색 그리드 크기 최적화 (`GRID_WIDTH`, `GRID_HEIGHT`)
- 모터 속도 제한 설정 (`setMaxSpeed`, `setMinSpeed`)
- 위치 업데이트 주기 조정 (`POSITION_UPDATE_INTERVAL`)

### 디버깅
- 시리얼 모니터를 통한 로그 확인
- 각 모듈별 상세한 디버그 메시지 제공
- API 응답을 통한 상태 확인
- 5초마다 자동 디버그 정보 출력

## 🔮 향후 개발 계획

### 하드웨어 확장
- **자이로스코프 연동**: 정밀한 방향 제어
- **배터리 모니터링**: 실시간 배터리 상태 추적
- **다중 초음파 센서**: 360도 장애물 감지
- **카메라 모듈**: 시각적 장애물 인식

### 소프트웨어 개선
- **머신러닝 통합**: 학습 기반 경로 최적화
- **클라우드 연동**: 원격 모니터링 및 제어
- **다중 로봇 협업**: 로봇 간 협력 시스템
- **모바일 앱**: 스마트폰 기반 제어

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🤝 기여하기

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📞 문의

프로젝트에 대한 문의사항이나 버그 리포트는 GitHub Issues를 통해 제출해주세요.

---

**SCV Robot** - 스마트한 자율주행 이동형 선반 로봇 🚀
