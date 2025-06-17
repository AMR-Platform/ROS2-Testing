// ─────────────────────────────────────────────────────────────────────────────
//  Stand-alone serial-reader + dead-reckoning (no ROS, no external libs)
//      • Packet format assumed identical to your Python script
//          H:<heading> R:<roll> P:<pitch> ... Enc:<left>/<right>\n
//      • Uses POSIX termios for the serial port (works on Linux / Jetson / macOS)
//      • Prints a line of CSV each time a complete packet is decoded:
//          heading_deg, roll_deg, pitch_deg, encL, encR, x[m], y[m], theta[deg]
// ─────────────────────────────────────────────────────────────────────────────

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

// ----- robot parameters (adapt to your chassis) -----------------------------
constexpr double WHEEL_BASE               = 0.30;      // m
constexpr int    ENCODER_TICKS_PER_REV    = 2048;
constexpr double WHEEL_DIAMETER           = 0.065;     // m
constexpr double TICKS_TO_DIST =
        (M_PI * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV;

// ----- helper: open a serial port with raw 8-N-1 --------------------------------
int open_serial(const char* device, int baud)
{
    int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); std::exit(1); }

    termios tty{}; tcgetattr(fd, &tty);
    cfmakeraw(&tty);

    speed_t br;
    switch (baud) {
        case 115200: br = B115200; break;
        case 57600:  br = B57600;  break;
        case 38400:  br = B38400;  break;
        default:
            std::cerr << "Unsupported baud\n"; std::exit(1);
    }
    cfsetispeed(&tty, br);
    cfsetospeed(&tty, br);

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem ctl lines, enable RX
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;               // 100 ms read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); }
    return fd;
}

// ----- helper: split by delimiter --------------------------------------------
std::vector<std::string_view> split_sv(std::string_view s, char delim)
{
    std::vector<std::string_view> out;
    std::size_t pos = 0;
    while (true)
    {
        std::size_t p = s.find(delim, pos);
        out.emplace_back(s.substr(pos, p == std::string_view::npos ? s.size()-pos : p-pos));
        if (p == std::string_view::npos) break;
        pos = p + 1;
    }
    return out;
}

// ----- main ------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_dev> <baud>\n";
        return 1;
    }
    const char* dev  = argv[1];
    int baud        = std::stoi(argv[2]);
    int fd          = open_serial(dev, baud);

    std::string line;
    line.reserve(128);

    long long prevEncL = -1, prevEncR = -1;
    double x = 0, y = 0, theta = 0;    // pose in metres / radians

    char buf[128];

    while (true)
    {
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n <= 0) { usleep(1000); continue; }

        for (ssize_t i = 0; i < n; ++i)
        {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                if (line.empty()) continue;

                //----------------------------------------------------------------
                //  Parse the finished line
                //----------------------------------------------------------------
                // quick sanity check
                if (line.find("H:") == std::string::npos ||
                    line.find("Enc:") == std::string::npos) {
                    line.clear();
                    continue;
                }

                // Heading, roll, pitch
                auto hPos = line.find("H:") + 2;
                auto rPos = line.find("R:");
                auto pPos = line.find("P:");
                auto vbPos = line.find("VB:");

                double heading = std::stod(line.substr(hPos, rPos - hPos));
                double roll    = std::stod(line.substr(rPos + 2, pPos - (rPos + 2)));
                double pitch   = std::stod(line.substr(pPos + 2, vbPos - (pPos + 2)));

                // Encoders
                auto encPos = line.find("Enc:") + 4;
                auto slash  = line.find('/', encPos);
                long long encL = std::stoll(line.substr(encPos, slash - encPos));
                long long encR = std::stoll(line.substr(slash + 1));

                //----------------------------------------------------------------
                //  Dead-reckoning
                //----------------------------------------------------------------
                if (prevEncL >= 0 && prevEncR >= 0) {
                    double dL = (encL - prevEncL) * TICKS_TO_DIST;
                    double dR = (encR - prevEncR) * TICKS_TO_DIST;
                    double dC = 0.5 * (dL + dR);
                    double dTh = (dR - dL) / WHEEL_BASE;

                    theta += dTh;
                    x += dC * std::cos(theta);
                    y += dC * std::sin(theta);
                }
                prevEncL = encL;  prevEncR = encR;

                //----------------------------------------------------------------
                //  Output
                //----------------------------------------------------------------
                std::cout << heading << ',' << roll << ',' << pitch << ','
                          << encL << ',' << encR << ','
                          << x << ',' << y << ',' << theta * 180.0 / M_PI
                          << '\n';

                line.clear();
            } else {
                line.push_back(c);
            }
        }
    }
    return 0;
}
