#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <stdlib.h>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

#include <chrono>
#include <map>

using namespace std::chrono;

class LogDuration {
public:
  explicit LogDuration(const string& msg = "")
    : message(msg + ": ")
    , start(steady_clock::now())
  {
  }

  ~LogDuration() {
    auto finish = steady_clock::now();
    auto dur = finish - start;
    cerr << message
       << duration_cast<milliseconds>(dur).count()
       << " ms" << endl;
  }
private:
  string message;
  steady_clock::time_point start;
};

#define UNIQ_ID_IMPL(lineno) _a_local_var_##lineno
#define UNIQ_ID(lineno) UNIQ_ID_IMPL(lineno)

#define LOG_DURATION(message) \
  LogDuration UNIQ_ID(__LINE__){message};

int randInt(int minValue, int maxValue) {
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<int> uniform_dist(minValue, maxValue);
    return uniform_dist(e1);
}

using CityMap = vector<string>;

constexpr char PositionFree = '.';
constexpr char PositionObstacle = '#';

struct Parameters {
    int N;
    int MaxTips;
    int Cost;
    CityMap map;
    int T;
    int D;
};

using ParametersPtr = shared_ptr<Parameters>;

istream& operator>>(istream& is, ParametersPtr pars) {
    is >> pars->N >> pars->MaxTips >> pars->Cost;
    pars->map.resize(pars->N);
    for (int i = 0; i < pars->N; i++) {
        pars->map[i].reserve(pars->N);
        is >> pars->map[i];
    }
    is >> pars->T >> pars->D;
    return is;
}

struct Position {
    int x = -1;
    int y = -1;
    bool valid(int N) const {
        return x >= 0 && y >= 0 && x < N && y < N;
    }
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
    Position getUpNeighbor() const {return {x, y-1};}
    Position getDownNeighbor() const {return {x, y+1};}
    Position getLeftNeighbor() const {return {x-1, y};}
    Position getRightNeighbor() const {return {x+1, y};}
};

struct PositionHasher {
    size_t operator()(const Position& pos) const {
        const int k = 37;
        return intHasher(pos.x) * k + pos.y;
    }
    std::hash<int> intHasher;
};

istream& operator>>(istream& is, Position& coord) {
    is >> coord.y >> coord.x;
    coord.y--;  // abnormal coords [1, N] instead of [0, N-1]
    coord.x--; // abnormal coords [1, N] instead of [0, N-1]
    return is;
}

ostream& operator<<(ostream& os, const Position& coord) {
    os << coord.y + 1 << " " << coord.x + 1; // abnormal coords [1, N] instead of [0, N-1]
    return os;
}

vector<vector<int>> computeDistanceMap(const CityMap& map, const Position& target) {
    int mapSize = static_cast<int>(map.size());
    vector<vector<int>> out(mapSize);
    for (auto& line : out)
        line.resize(mapSize, -2);
    queue<pair<Position, int>> fifo;
    fifo.push({target, 0});
    while (!fifo.empty()) {
        auto node = fifo.front();
        fifo.pop();
        auto& position = node.first;
        int distance = node.second;
        out[position.y][position.x] = distance;
        vector<Position> adjacentVertices = {
            position.getUpNeighbor(), position.getDownNeighbor(),
            position.getLeftNeighbor(), position.getRightNeighbor()
        };
        for (auto& nextVertex: adjacentVertices) {
            if (nextVertex.valid(mapSize)) {
                bool visited = out[nextVertex.y][nextVertex.x] != -2;
                if ((!visited) && (map[nextVertex.y][nextVertex.x] == PositionFree)) {
                    fifo.push({nextVertex, distance + 1});
                    out[nextVertex.y][nextVertex.x] = -1;
                }
            }
        }
    }
    return out;
}

vector<Position> findShortestPath(const CityMap& map, const Position start, const Position target) {
    queue<Position> fifo;
    int map_size = static_cast<int>(map.size());
    vector<vector<Position>> visited(map_size);
    for (auto& line : visited)
        line.resize(map_size);
    vector<Position> path;
    fifo.push(start);
    while (fifo.size()) {
        auto v = fifo.front();
        fifo.pop();
        vector<Position> adjacent_vertices = {{v.x-1, v.y}, {v.x+1, v.y}, {v.x, v.y-1}, {v.x, v.y+1}}; 
        for (auto next_vertex: adjacent_vertices) {
            if (next_vertex == target) {
                visited[next_vertex.y][next_vertex.x] = {v.x, v.y};
                fifo.push(next_vertex);
                break;
            }
            if (next_vertex.valid(map_size)) {
                bool visited_flag = visited[next_vertex.y][next_vertex.x].valid(map_size);
                auto vertex_value = map[next_vertex.y][next_vertex.x];
                if (!visited_flag && (vertex_value == PositionFree)) {
                    visited[next_vertex.y][next_vertex.x] = {v.x, v.y};
                    fifo.push(next_vertex);
                }
            }
        }
        if (fifo.back() == target) {
            path.push_back(target);
            while (!(path.back() == start)) {
                auto next_vertex = visited[path.back().y][path.back().x];
                path.push_back(next_vertex);
            }
            break;
        }
    }
    reverse(path.begin(), path.end());
    return path;
}

enum class RoverState {
    st_idle,
    st_goToStart,
    st_goToDest,
};

enum class RoverMotion {
    up, down, left, right, stay, pick, issue, 
};

char action2symbol(RoverMotion motion) {
    switch (motion)
    {
    case RoverMotion::stay: return 'S';
    case RoverMotion::up: return 'U';
    case RoverMotion::down: return 'D';
    case RoverMotion::left: return 'L';
    case RoverMotion::right: return 'R';
    case RoverMotion::pick: return 'T';
    case RoverMotion::issue: return 'P';
    default: throw runtime_error("Unexpected rover motion gotten by action2symbol()");
    }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

using DistanceMap = vector<vector<int>>;

class Delivery {
public:
    Delivery(Position from, Position to, const CityMap& map) :
        _start(from), _target(to) {
        _distToStart = make_unique<DistanceMap>();
        _distToDestination = make_unique<DistanceMap>();
        *_distToStart = computeDistanceMap(map, from);
        *_distToDestination = computeDistanceMap(map, to);
    }
    Position getStart() const {return _start;}
    Position getDestination() const {return _target;}
    int getAge() const {return _age;}
    void refreshAge(int seconds = 1) {_age += seconds;}
    const DistanceMap& getDistMapToStart() const {return *_distToStart;}
    const DistanceMap& getDistMapToDestination() const {return *_distToDestination;}
    void eraseDistanceMaps() {
        _distToStart = nullptr;
        _distToDestination = nullptr;
    }
private:
    Position _start;
    Position _target;
    int _age = 0;
    unique_ptr<DistanceMap> _distToStart;
    unique_ptr<DistanceMap> _distToDestination;
};

using DeliveryPtr = shared_ptr<Delivery>;
using OrdersStorage = std::unordered_map<Position, deque<DeliveryPtr>, PositionHasher>;
//using DeliveryId = OrdersStorage::iterator;

class Rover {
public:   
    Rover(const CityMap& map, OrdersStorage& orders, unordered_set<DeliveryPtr>& ordersInProgress) 
        : _map(map), _orders(orders), _ordersInProgress(ordersInProgress) {
        int mapSize = static_cast<int>(_map.size());
        bool freePlace = false;
        while (!freePlace) {
            _position.x = randInt(0, mapSize-1);
            _position.y = randInt(0, mapSize-1);
            freePlace = _map[_position.y][_position.x] == PositionFree;
        }
        _actions.reserve(60);
    }
    Position getLocation() const {return _position;}
    bool isFree() const {return _schedule.empty();}
    void setOrder(DeliveryPtr order) {
        _order = order;
        scheduleTrek(_order);
        _order->eraseDistanceMaps();
    }
    void scheduleTrek(DeliveryPtr order) {
        int mapSize = static_cast<int>(order->getDistMapToStart().size());
        Position virtualRover = _position;
        const DistanceMap& distMapToStart = order->getDistMapToStart();
        const DistanceMap& distMapToDest = order->getDistMapToDestination();
        while (!(virtualRover == order->getStart())) {
            vector<pair<Position, RoverMotion>> options = {
                {virtualRover.getUpNeighbor(), RoverMotion::up},
                {virtualRover.getDownNeighbor(), RoverMotion::down},
                {virtualRover.getLeftNeighbor(), RoverMotion::left},
                {virtualRover.getRightNeighbor(), RoverMotion::right}
            };
            int currentDistance = distMapToStart[virtualRover.y][virtualRover.x];
            for (const auto& option: options) {
                const auto& neighbor = option.first;
                if (neighbor.valid(mapSize) && (distMapToStart[neighbor.y][neighbor.x] >= 0)) {
                    if (distMapToStart[neighbor.y][neighbor.x] < currentDistance) {
                        _schedule.push(option.second);
                        virtualRover = neighbor;
                        break;
                    }
                }
            }
        }
        _schedule.push(RoverMotion::pick);
        while (!(virtualRover == order->getDestination())) {
            vector<pair<Position, RoverMotion>> options = {
                {virtualRover.getUpNeighbor(), RoverMotion::up},
                {virtualRover.getDownNeighbor(), RoverMotion::down},
                {virtualRover.getLeftNeighbor(), RoverMotion::left},
                {virtualRover.getRightNeighbor(), RoverMotion::right}
            };
            int currentDistance = distMapToDest[virtualRover.y][virtualRover.x];
            for (const auto& option: options) {
                const auto& neighbor = option.first;
                if (neighbor.valid(mapSize) && (distMapToDest[neighbor.y][neighbor.x] >= 0)) {
                    if (distMapToDest[neighbor.y][neighbor.x] < currentDistance) {
                        _schedule.push(option.second);
                        virtualRover = neighbor;
                        break;
                    }
                }
            }
        }
        _schedule.push(RoverMotion::issue);
    } 
    void go(RoverMotion motion) {
        switch (motion)
        {
        case RoverMotion::down:
            _position = _position.getDownNeighbor();
            break;
        case RoverMotion::up:
            _position = _position.getUpNeighbor();
            break;
        case RoverMotion::left:
            _position = _position.getLeftNeighbor();
            break;
        case RoverMotion::right:
            _position = _position.getRightNeighbor();
            break;                    
        default:
            break;
        }
    }
    void makeNextMove() {
        if (_schedule.empty())
            _actions.push_back(action2symbol(RoverMotion::stay));
        else {
            _actions.push_back(action2symbol(_schedule.front()));
            go(_schedule.front());
            _schedule.pop();
            if (_schedule.empty()) {
                _ordersInProgress.erase(_order);
                _orders[_order->getStart()].pop_front();
            }
        }
    }
    const string& getActions() const {
        return _actions;
    };
    void resetActions() {
        _actions.clear();
        _actions.reserve(60);
    }
    int reward(DeliveryPtr order, int maxTips) const {
        if (order->getAge() >= maxTips)
            return 0;
        const auto& deliveryStart = order->getStart();
        int pathToStart = order->getDistMapToStart()[_position.y][_position.x];
        int deliveryPath = order->getDistMapToDestination()[deliveryStart.y][deliveryStart.x];
        return std::max(0, maxTips - (pathToStart + deliveryPath + order->getAge()));
    }
private:
    Position _position;
    const CityMap& _map;
    string _actions;
    DeliveryPtr _order;
    std::queue<RoverMotion> _schedule;

    OrdersStorage& _orders;
    unordered_set<DeliveryPtr>& _ordersInProgress;
};

class RoversController {
public:
    RoversController(ParametersPtr data, istream& is = cin, 
        ostream& os = cout) :
            _pars(data), _is(is), _os(os) {
        generateRovers();
        run();
    }    
private:
    int findOptimalRoversCnt() {
        int maxRoverCnt = std::min(100., std::round(1. * _pars->D * _pars->MaxTips / _pars->Cost));
        int freeSpaceArea = 0;
        for (const auto& line : _pars->map) {
            freeSpaceArea += std::count_if(line.begin(), line.end(), [] (const char& c) {return c == PositionFree;});
        }
        int minRoverCnt = std::max(1., freeSpaceArea / (std::pow(0.5 * _pars->MaxTips, 2) + 1e-6));
        if (minRoverCnt > maxRoverCnt) {
            if (maxRoverCnt > 1)
                return randInt(1, maxRoverCnt);
            else
                return 1;
        } else {
            return randInt(minRoverCnt, maxRoverCnt);
        }
    }
    void generateRovers() {
        int roversCnt = findOptimalRoversCnt();
        _rovers.reserve(roversCnt);
        for (int i = 0; i < roversCnt; i++)
            _rovers.emplace_back(Rover(_pars->map, _orders, _ordersInProgress));
        _os << _rovers.size() << "\n";
        for (const auto& rover: _rovers)
            _os << rover.getLocation() << "\n"; 
        _os.flush();
    }
    DeliveryPtr findBestOrder(const Rover& rover) {
        //LOG_DURATION("RoversController::findBestOrder");
        int maxTips = _pars->MaxTips;
        vector<DeliveryPtr> actualOrders;
        for (auto it = _orders.begin(); it != _orders.end(); it++)
            if (!it->second.empty()) {
                if (!_ordersInProgress.count(it->second.front()))
                    actualOrders.push_back(it->second.front());
            }
        if (actualOrders.empty())
            return nullptr;
        auto bestOrderIt = std::max_element(actualOrders.begin(), actualOrders.end(), 
            [&rover, maxTips] (const auto& lhs, const auto& rhs) {
                return rover.reward(lhs, maxTips) > rover.reward(rhs, maxTips);
            });
        return *bestOrderIt;
    }
    void run() {
        for (int iterId = 0; iterId < _pars->T; iterId++) {
            //LOG_DURATION("RoversController::run()");
            // update orders
            int k; _is >> k;
            for (int orderId = 0; orderId < k; orderId++) {
                Position orderStart, orderStop;
                _is >> orderStart >> orderStop;
                auto order = std::make_shared<Delivery>(orderStart, orderStop, _pars->map);
                _orders[order->getStart()].push_back(std::move(order));
            }
            // run robots
            for (int second = 0; second < 60; second ++) {
                for (auto& rover : _rovers) {
                    if (rover.isFree()) {
                        // find new order
                        DeliveryPtr bestOrder = findBestOrder(rover);
                        if (bestOrder != nullptr) {
                            rover.setOrder(bestOrder);
                            _ordersInProgress.insert(bestOrder);
                        }
                    }
                    rover.makeNextMove();
                }
                for (auto& posData: _orders) {
                    auto& deliveryQueue = posData.second;
                    for (auto& order: deliveryQueue)
                        order->refreshAge(1);
                }
            }
            for (auto& rover : _rovers) {
                _os << rover.getActions() << "\n";
                rover.resetActions();
            }
            _os.flush();
        }
    }

    ParametersPtr _pars;
    istream& _is;
    ostream& _os;

    vector<Rover> _rovers;
    OrdersStorage _orders;
    unordered_set<DeliveryPtr> _ordersInProgress;
};

template<typename T>
ostream& operator<<(ostream& os, const vector<vector<T>>& matrix) {
    for (const auto& line : matrix) {
        for (const T& p : line)
            os << p;
        os << "\n";
    }
    return os;
}

int main(int argc, char** argv) {
    ParametersPtr pars = make_shared<Parameters>();
    if (argc > 1) {
         ifstream is(argv[1]);
         is >> pars;
         RoversController solution(pars, is, cout);
    } else {
         cin >> pars;
         RoversController solution(pars, cin, cout);
    }
    return 0;
}
