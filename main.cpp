#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <random>
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

int randInt(int min, int max) {
    return min + std::rand() % (max - min);
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
    bool valid(int N) {
        return x >= 0 && y >= 0 && x < N && y < N;
    }
    bool operator==(const Position& other) {
        return x == other.x && y == other.y;
    }
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

class Delivery {
public:
    Delivery(Position from, Position to) :
        _start(from), _target(to) {}
    Position getStart() const {return _start;}
    Position getDestination() const {return _target;}
    int getAge() const {return _age;}
    void refreshAge(int seconds = 1) {_age += seconds;}
private:
    Position _start;
    Position _target;
    int _age = 0;
};

using OrderIt = list<Delivery>::iterator;

class Rover {
public:   
    Rover(const CityMap& map) 
        : _map(map) {
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
    bool isFree() const {return _state == RoverState::st_idle;}
    void setOrder(std::shared_ptr<Delivery> order) {
        _order = order;
        _state = RoverState::st_goToStart;
        _route = findShortestPath(_map, _position, _order->getStart());
        _step = 0;
    }
    RoverMotion defineMotion(const Position& from, const Position& to) {
        if (from.x == to.x && from.y == to.y)
            return RoverMotion::stay;
        else if (from.x == to.x) {
            return to.y < from.y ? RoverMotion::up : RoverMotion::down;
        } else if (from.y == to.y) {
            return to.x < from.x ? RoverMotion::left : RoverMotion::right;
        } else {
            throw runtime_error("defineMotion() got not adjacent points");
        }
    }
    void makeNextMove() {
        switch (_state)
        {
            case RoverState::st_idle:
                _actions.push_back(action2symbol(RoverMotion::stay));
                break;
            case RoverState::st_goToStart:
                if (_position == _order->getStart()) {
                    _actions.push_back(action2symbol(RoverMotion::pick));
                    _state = RoverState::st_goToDest;
                    _route = findShortestPath(_map, _order->getStart(), _order->getDestination());
                    _step = 0;
                } else {
                    RoverMotion nextMotion = defineMotion(_route[_step],_route[_step+1]);
                    _actions.push_back(action2symbol(nextMotion));
                    _position = _route[_step+1];
                    _step++;
                }
                break;
            case RoverState::st_goToDest:
                if (_position == _order->getDestination()) {
                    _actions.push_back(action2symbol(RoverMotion::issue));
                    _state = RoverState::st_idle;
                    _route.clear();
                    _step = 0;
                } else {
                    RoverMotion nextMotion = defineMotion(_route[_step],_route[_step+1]);
                    _actions.push_back(action2symbol(nextMotion));
                    _position = _route[_step+1];
                    _step++;
                }
                break;
            default:
                throw runtime_error("Unexpected rover state in Rover::makeNextMove()");
                break;
        }
    }
    const string& getActions() const {
        return _actions;
    };
    void resetActions() {
        _actions.clear();
        _actions.reserve(60);
    }
    int reward(const Delivery& order, int maxTips) const {
        if (order.getAge() >= maxTips)
            return 0;
        int pathToStart = findShortestPath(_map, _position, order.getStart()).size();
        int deliveryPath = findShortestPath(_map, order.getStart(), order.getDestination()).size();
        return std::max(0, maxTips - (pathToStart + deliveryPath + order.getAge()));
    }
private:
    Position _position;
    vector<Position> _route;
    int _step;
    RoverState _state = RoverState::st_idle;
    const CityMap& _map;
    string _actions;
    std::shared_ptr<Delivery> _order;
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
        int averageSimOrdersCnt = std::round(_pars->D / _pars->T);
        return std::min(maxRoverCnt, std::max(1, averageSimOrdersCnt));
    }
    void generateRovers() {
        int roversCnt = findOptimalRoversCnt();
        _rovers.reserve(roversCnt);
        for (int i = 0; i < roversCnt; i++)
            _rovers.emplace_back(Rover(_pars->map));
        _os << _rovers.size() << "\n";
        for (const auto& rover: _rovers)
            _os << rover.getLocation() << "\n"; 
        _os.flush();
    }
    list<Delivery>::iterator findBestOrder(const Rover& rover) {
        LOG_DURATION("RoversController::findBestOrder");
        int maxTips = _pars->MaxTips;
        return std::max_element(_orders.begin(), _orders.end(), 
            [&rover, maxTips] (const Delivery& lhs, const Delivery& rhs) {
                return rover.reward(lhs, maxTips) > rover.reward(rhs, maxTips);
            });
    }
    void run() {
        for (int iterId = 0; iterId < _pars->T; iterId++) {
            LOG_DURATION("RoversController::run()");
            // update orders
            int k; _is >> k;
            for (int orderId = 0; orderId < k; orderId++) {
                Position orderStart, orderStop;
                _is >> orderStart >> orderStop;
                _orders.push_back({orderStart, orderStop});
            }
            // run robots
            for (int second = 0; second < 60; second ++) {
                for (auto& rover : _rovers) {
                    if (rover.isFree()) {
                        // find new order
                        OrderIt bestOrder = findBestOrder(rover);
                        if (bestOrder != _orders.end()) {
                            auto order = make_shared<Delivery>(*bestOrder);
                            _orders.erase(bestOrder);
                            rover.setOrder(order);
                        }
                    }
                    rover.makeNextMove();
                }
                for (auto& order: _orders)
                    order.refreshAge(1);
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
    list<Delivery> _orders;
};

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
