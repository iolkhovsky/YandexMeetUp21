#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace std;

using CityMap = std::vector<std::string>;

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

class RoverController {
    public:
        RoverController(DataPtr data, std::istream& is = std::cin, std::ostream& os = std::cout) :
        _data(data), _is(is), _os(os) {}
    private:
        DataPtr _data;
        std::istream& _is;
        std::ostream& _os;
};

int main(int argc, char** argv) {
    DataPtr pars = std::make_shared<InputData>();
    if (argc > 1) {
        cout << "Reading input data from " << argv[1] << "\n";
        ifstream is(argv[1]);
        is >> pars;
    } else {
        cout << "Reading input data from std::cin" << "\n";
        std::cin >> pars;
    }
    cout << "Loaded data:" << "\n" << pars << pars->map;

    return 0;
}