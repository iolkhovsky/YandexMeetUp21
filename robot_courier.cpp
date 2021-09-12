#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

using namespace std;

using CityMap = std::vector<std::string>;
constexpr char Free = '.';
constexpr char Obstacle = '#';

struct InputData {
    int city_size;
    int max_tips;
    int cost;
    CityMap map;
    int iterations;
    int orders_cnt;
};

using DataPtr = std::shared_ptr<InputData>;

std::istream& operator>>(std::istream& is, DataPtr data) {
    is >> data->city_size >> data->max_tips >> data->cost;
    data->map.resize(data->city_size);
    for (int i = 0; i < data->city_size; i++) {
        data->map[i].reserve(data->city_size);
        is >> data->map[i];
    }
    is >> data->iterations >> data->orders_cnt;
    return is;
}

std::ostream& operator<<(std::ostream& os, DataPtr data) {
    os << "City size (N): " << data->city_size << "\n";
    os << "Max tips (MaxTips): " << data->max_tips << "\n";
    os << "Rover cost (Costc): " << data->cost << "\n";
    os << "Iterations (T): " << data->iterations << "\n";
    os << "Orders count (D): " << data->orders_cnt << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const CityMap& map) {
    os << "City map: " << "\n";
    for (const auto& line : map)
        os << line << "\n";
    return os;
}

struct Coordinate {
    int x = -1;
    int y = -1;
    bool valid(int N) {
        return x >= 0 && y >= 0 && x < N && y < N;
    }
    bool operator==(const Coordinate other) {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Coordinate other) {
        return x != other.x || y != other.y;
    }
};

class Rover {
public:
    enum class States {
        st_idle,
        st_pickup,
        st_delivery,
    };
    Rover() = default;
private:
    Coordinate _position;
    Coordinate _delivery_start;
    Coordinate _delivery_target;
};

std::vector<Coordinate> findShortestPath(const CityMap& map, const Coordinate start, const Coordinate target) {
    std::queue<Coordinate> fifo;
    int map_size = static_cast<int>(map.size());
    std::vector<std::vector<Coordinate>> visited(map_size);
    for (auto& line : visited)
        line.resize(map_size);
    std::vector<Coordinate> path;
    fifo.push(start);
    while (fifo.size()) {
        auto v = fifo.front();
        fifo.pop();
        std::vector<Coordinate> adjacent_vertices = {{v.x-1, v.y}, {v.x+1, v.y}, {v.x, v.y-1}, {v.x, v.y+1}}; 
        for (auto next_vertex: adjacent_vertices) {
            if (next_vertex == target) {
                visited[next_vertex.y][next_vertex.x] = {v.x, v.y};
                fifo.push(next_vertex);
                break;
            }
            if (next_vertex.valid(map_size)) {
                bool visited_flag = visited[next_vertex.y][next_vertex.x].valid(map_size);
                auto vertex_value = map[next_vertex.y][next_vertex.x];
                if (!visited_flag && (vertex_value == Free)) {
                    visited[next_vertex.y][next_vertex.x] = {v.x, v.y};
                    fifo.push(next_vertex);
                }
            }
        }
        if (fifo.back() == target) {
            path.push_back(target);
            while (path.back() != start) {
                auto next_vertex = visited[path.back().y][path.back().x];
                path.push_back(next_vertex);
            }
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

class RoverController {
    public:
        RoverController(DataPtr data, std::istream& is = std::cin, std::ostream& os = std::cout) :
        _data(data), _is(is), _os(os) {
            generateGraph();
            generateRovers();
        }
    private:
        void generateRovers() {
            int rovers_cnt = std::min(100., std::max(1., std::round(_data->orders_cnt / _data->iterations)));
            _rovers.reserve(rovers_cnt);
            for (int i = 0; i < rovers_cnt; i++)
                _rovers.push_back(Rover());
            _os << _rovers.size();
            _os.flush();
        }
        void generateGraph() {

        }

        DataPtr _data;
        std::istream& _is;
        std::ostream& _os;
        std::vector<Rover> _rovers;
        
};

int main(int argc, char** argv) {
    DataPtr pars = std::make_shared<InputData>();
    if (argc > 1) {
        std::cout << "Reading input data from " << argv[1] << "\n";
        ifstream is(argv[1]);
        is >> pars;
    } else {
        std::cout << "Reading input data from std::cin" << "\n";
        std::cin >> pars;
    }
    cout << "Loaded data:" << "\n" << pars << pars->map;

    RoverController solution(pars);

    cout << "Shortest path:\n";
    for (auto p: findShortestPath(pars->map, {1, 3}, {3, 3}))
        std::cout << p.x << " " << p.y << "\n";

    return 0;
}