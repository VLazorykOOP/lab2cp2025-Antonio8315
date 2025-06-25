#include <iostream>
#include <thread>
#include <mutex>
#include <random>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std;
using namespace std::chrono;

constexpr double V = 1.0;
constexpr int DT_MS = 1000;
constexpr int N = 3;
constexpr int SIM_TIME_SEC = 10;

mutex cout_mtx;

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
void safePrint(const string &s)
{
    lock_guard<mutex> lk(cout_mtx);
    cout << s << '\n';
}

class WorkerBee
{
    double x, y, homeX, homeY;
    bool toOrigin = true;

public:
    WorkerBee(double sx, double sy) : x(sx), y(sy), homeX(sx), homeY(sy) {}
    void operator()()
    {
        const double step = V * DT_MS / 1000.0;
        while (true)
        {
            double tx = toOrigin ? 0.0 : homeX;
            double ty = toOrigin ? 0.0 : homeY;
            double d = dist(x, y, tx, ty);
            if (d <= step)
            {
                x = tx;
                y = ty;
                toOrigin = !toOrigin;
            }
            else
            {
                x += step * (tx - x) / d;
                y += step * (ty - y) / d;
            }
            safePrint("Worker (" + to_string(homeX) + "," + to_string(homeY) + ") at " + to_string(x) + ", " + to_string(y));
            this_thread::sleep_for(milliseconds(DT_MS));
        }
    }
};

class Drone
{
    double x, y, dirX, dirY;
    mt19937 gen{random_device{}()};
    uniform_real_distribution<double> ang{0, 2 * M_PI};

public:
    Drone(double sx, double sy) : x(sx), y(sy) { changeDir(); }
    void changeDir()
    {
        double a = ang(gen);
        dirX = cos(a);
        dirY = sin(a);
    }
    void operator()()
    {
        const double step = V * DT_MS / 1000.0;
        int msToTurn = N * 1000;
        while (true)
        {
            if (msToTurn <= 0)
            {
                changeDir();
                msToTurn = N * 1000;
            }
            x += dirX * step;
            y += dirY * step;
            safePrint("Drone at " + to_string(x) + ", " + to_string(y));
            this_thread::sleep_for(milliseconds(DT_MS));
            msToTurn -= DT_MS;
        }
    }
};

int main()
{
    vector<thread> bees;
    bees.emplace_back(WorkerBee(2.0, 3.0));
    bees.emplace_back(WorkerBee(-3.0, -3.0));
    bees.emplace_back(Drone(0.0, 0.0));
    bees.emplace_back(Drone(4.0, 1.0));

    this_thread::sleep_for(seconds(SIM_TIME_SEC));

    safePrint("\n=== STOP ===");
    for (auto &t : bees)
        t.detach();
    return 0;
}
