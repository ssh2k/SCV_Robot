#ifndef MAP_LEARNER_H
#define MAP_LEARNER_H

#include "pathfinder.h"

class MapLearner {
public:
    MapLearner(Pathfinder* pathfinder, int gridWidth, int gridHeight);
    void begin();
    void learnMap(); // 맵 학습 시작
    void applyLearnedMap(); // 학습된 맵을 pathfinder에 반영

private:
    Pathfinder* pathfinder;
    int gridWidth, gridHeight;
    bool** learnedGrid; // 장애물 정보 저장

    void scanAtGrid(int gridX, int gridY);
    float getUltrasonicDistance(int angleDeg); // 실제 센서 연동 필요
    void markObstacle(int gridX, int gridY);
    void clearLearnedGrid();
};

#endif 