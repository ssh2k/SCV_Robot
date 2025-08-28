#include "explorer.h"
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>

namespace Explorer {

// Point, Direction declarations are in explorer.h

// 2D Grid Map
static constexpr int WIDTH = 50;
static constexpr int HEIGHT = 50;
// Cell: 0 = Unknown, 1 = Wall, 2 = Free
static int grid[HEIGHT][WIDTH];

// Beacons (can be updated via setBeacons)
static std::vector<Point> beacons = {
    {10, 15}, {25, 30}, {40, 20}
};

static Direction currentDirection = Direction::Up;

void initializeGrid(int defaultValue) {
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            grid[y][x] = defaultValue;
        }
    }
}

void setBeacons(const std::vector<Point>& newBeacons) {
    beacons = newBeacons;
}

bool inBounds(const Point& p) {
    return !(p.x < 0 || p.x >= WIDTH || p.y < 0 || p.y >= HEIGHT);
}

bool detectWall(const Point& pos) {
    if (!inBounds(pos)) return true;
    return grid[pos.y][pos.x] == 1;
}

Point dirVector(Direction d) {
    switch (d) {
        case Direction::Up: return {0, -1};
        case Direction::Down: return {0, 1};
        case Direction::Left: return {-1, 0};
        case Direction::Right: return {1, 0};
    }
    return {0, 0};
}

Direction leftOf(Direction d) {
    switch (d) {
        case Direction::Up: return Direction::Left;
        case Direction::Left: return Direction::Down;
        case Direction::Down: return Direction::Right;
        case Direction::Right: return Direction::Up;
    }
    return Direction::Up;
}

Direction rightOf(Direction d) {
    switch (d) {
        case Direction::Up: return Direction::Right;
        case Direction::Right: return Direction::Down;
        case Direction::Down: return Direction::Left;
        case Direction::Left: return Direction::Up;
    }
    return Direction::Up;
}

// Left-wall following exploration. Marks visited free cells as 2.
void exploreMap(const Point& start, int maxSteps = 1000) {
    Point robot = start;
    std::set<std::pair<int, int>> visited;
    if (inBounds(robot)) {
        grid[robot.y][robot.x] = 2;
        visited.insert({robot.x, robot.y});
    }

    for (int steps = 0; steps < maxSteps; ++steps) {
        Direction leftDir = leftOf(currentDirection);
        Point leftPos = {robot.x + dirVector(leftDir).x, robot.y + dirVector(leftDir).y};

        if (!detectWall(leftPos)) {
            currentDirection = leftDir;
            Point nextPos = {robot.x + dirVector(currentDirection).x, robot.y + dirVector(currentDirection).y};
            if (!detectWall(nextPos) && inBounds(nextPos)) {
                robot = nextPos;
                grid[robot.y][robot.x] = 2;
                visited.insert({robot.x, robot.y});
            } else {
                currentDirection = rightOf(currentDirection);
            }
        } else {
            Point forwardPos = {robot.x + dirVector(currentDirection).x, robot.y + dirVector(currentDirection).y};
            if (!detectWall(forwardPos) && inBounds(forwardPos)) {
                robot = forwardPos;
                grid[robot.y][robot.x] = 2;
                visited.insert({robot.x, robot.y});
            } else {
                currentDirection = rightOf(currentDirection);
            }
        }

        if (robot.x == start.x && robot.y == start.y && steps > 1) {
            break;
        }
    }
}

struct Node {
    Point p;
    int g;
    int h;
    bool operator<(const Node& other) const {
        return (g + h) > (other.g + other.h);
    }
};

int heuristic(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// A*: If allowUnknown == false, treat unknown cells as blocked. Only free(2) is traversable.
std::vector<Point> aStar(const Point& start, const Point& goal, bool allowUnknown = false) {
    if (!inBounds(start) || !inBounds(goal)) return {};

    std::priority_queue<Node> open;
    std::set<std::pair<int, int>> closed;
    std::map<std::pair<int, int>, std::pair<int, int>> cameFrom;
    int inf = std::numeric_limits<int>::max();
    int gStart = 0;

    std::vector<std::vector<int>> gScore(HEIGHT, std::vector<int>(WIDTH, inf));
    gScore[start.y][start.x] = 0;

    open.push({start, gStart, heuristic(start, goal)});

    auto reconstruct = [&](Point cur) {
        std::vector<Point> path;
        std::pair<int, int> pos = {cur.x, cur.y};
        while (pos.first != start.x || pos.second != start.y) {
            path.push_back({pos.first, pos.second});
            pos = cameFrom[pos];
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (cur.p.x == goal.x && cur.p.y == goal.y) {
            return reconstruct(cur.p);
        }

        if (closed.count({cur.p.x, cur.p.y})) continue;
        closed.insert({cur.p.x, cur.p.y});

        Direction dirs[4] = {Direction::Up, Direction::Right, Direction::Down, Direction::Left};
        for (Direction d : dirs) {
            Point np = {cur.p.x + dirVector(d).x, cur.p.y + dirVector(d).y};
            if (!inBounds(np)) continue;

            if (grid[np.y][np.x] == 1) continue; // Wall
            if (!allowUnknown && grid[np.y][np.x] != 2) continue; // Unknown blocked when not allowed

            if (closed.count({np.x, np.y})) continue;

            int tentative = cur.g + 1;
            if (tentative < gScore[np.y][np.x]) {
                gScore[np.y][np.x] = tentative;
                cameFrom[{np.x, np.y}] = {cur.p.x, cur.p.y};
                open.push({np, tentative, heuristic(np, goal)});
            }
        }
    }
    return {};
}

// Utility: compute path lengths from start to each beacon.
std::vector<int> computePathLengthsToBeacons(const Point& start, bool allowUnknown = false) {
    std::vector<int> lengths;
    lengths.reserve(beacons.size());
    for (const auto& b : beacons) {
        auto path = aStar(start, b, allowUnknown);
        lengths.push_back(static_cast<int>(path.size()));
    }
    return lengths;
}

// Accessors
int width() { return WIDTH; }
int height() { return HEIGHT; }
int getCell(int x, int y) { return inBounds({x, y}) ? grid[y][x] : 1; }
void setCell(int x, int y, int value) { if (inBounds({x, y})) grid[y][x] = value; }
Direction getDirection() { return currentDirection; }
void setDirection(Direction d) { currentDirection = d; }

// Convert Explorer grid (2=free,1=wall,0=unknown) to Pathfinder grid (0=free,1=blocked)
std::vector<std::vector<int>> exportGridForPathfinder() {
    std::vector<std::vector<int>> out(HEIGHT, std::vector<int>(WIDTH, 1));
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            out[y][x] = (grid[y][x] == 2) ? 0 : 1;
        }
    }
    return out;
}

} // namespace Explorer


