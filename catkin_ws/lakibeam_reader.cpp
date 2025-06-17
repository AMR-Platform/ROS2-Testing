// lakibeam_reader.cpp
// -----------------------------------------------------------------------------
// Build: g++ -std=c++17 -O2 lakibeam_reader.cpp -o lakibeam_reader
// Run  : ./lakibeam_reader --sensorip 192.168.8.2 --port 2368
// -----------------------------------------------------------------------------

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>


static constexpr std::size_t  PKT_SIZE = 1248;     // bytes / packet
static constexpr std::size_t  PKT_HDR  = 42;       // Ethernet+IP+UDP header
static constexpr std::size_t  BLOCK_SZ = 100;
static constexpr int          BLOCKS   = 12;
static constexpr int          CH_PER_BLOCK = 32;
static constexpr double       RES_DIST = 0.01;     // m / count
static constexpr double       RES_AZ   = 0.25;     // ° step inside block

#pragma pack(push,1)
struct BlockHeader {
    uint16_t flag;        // should be 0xEEFF (little-endian)
    uint16_t azimuth_hud; // azimuth * 100
};
#pragma pack(pop)

struct CLI {
    std::string hostip   = "0.0.0.0";
    std::string sensorip = "";          // “discard others” if set
    int   port           = 2368;
    double angle_offset  = 0.0;
    double a_start       = 45.0;
    double a_stop        = 315.0;
    bool   inverted      = false;
};

CLI parse_cli(int argc, char** argv)
{
    CLI opt;
    for (int i = 1; i < argc; ++i) {
        auto arg = std::string(argv[i]);
        auto next = [&](const char* flg) -> std::string {
            if (i + 1 >= argc) { std::cerr << flg << " needs value\n"; exit(1); }
            return argv[++i];
        };
        if (arg == "--hostip")        opt.hostip   = next(arg.c_str());
        else if (arg == "--sensorip") opt.sensorip = next(arg.c_str());
        else if (arg == "--port")     opt.port     = std::stoi(next(arg.c_str()));
        else if (arg == "--angle_offset") opt.angle_offset = std::stod(next(arg.c_str()));
        else if (arg == "--scan_range_start") opt.a_start   = std::stod(next(arg.c_str()));
        else if (arg == "--scan_range_stop")  opt.a_stop    = std::stod(next(arg.c_str()));
        else if (arg == "--inverted")  opt.inverted = true;
        else { std::cerr << "Unknown arg " << arg << '\n'; exit(1); }
    }
    return opt;
}

inline bool in_range(double az, double a0, double a1)
{
    if (a0 < a1) return az >= a0 && az <= a1;
    return az >= a0 || az <= a1;          // wrapped window (e.g. 300-60)
}

int main(int argc, char** argv)
{
    CLI cfg = parse_cli(argc, argv);

    // ───────────── UDP socket ─────────────
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(cfg.port);
    addr.sin_addr.s_addr = inet_addr(cfg.hostip.c_str());
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    { perror("bind"); return 1; }

    // non-blocking
    fcntl(fd, F_SETFL, O_NONBLOCK);

    std::unordered_map<double, std::pair<double,uint8_t>> scan; // az → {dist,rssi}
    double last_az = -1.0;
    std::array<uint8_t, PKT_SIZE> pkt{};

    std::cout << "Listening on " << cfg.hostip << ':' << cfg.port << std::endl;

    while (true)
    {
        sockaddr_in src{};
        socklen_t sl = sizeof(src);
        ssize_t n = ::recvfrom(fd, pkt.data(), PKT_SIZE, 0,
                               reinterpret_cast<sockaddr*>(&src), &sl);
        if (n < 0) {                          // nothing ready
            usleep(1000);
            continue;
        }
        if (n != PKT_SIZE) continue;
        if (!cfg.sensorip.empty() &&
            cfg.sensorip != inet_ntoa(src.sin_addr)) continue;

        for (int b = 0; b < BLOCKS; ++b) {
            const uint8_t* blk = pkt.data() + PKT_HDR + b*BLOCK_SZ;
            const auto* bh = reinterpret_cast<const BlockHeader*>(blk);
            if (bh->flag != 0xFFEE) continue;

            double az0 = bh->azimuth_hud / 100.0; // hundredths deg → deg
            for (int ch = 0; ch < CH_PER_BLOCK; ++ch) {
                const uint8_t* triad = blk + 4 + ch*3;
                uint16_t dist_raw = triad[0] | (triad[1] << 8);
                uint8_t  rssi     = triad[2];
                if (dist_raw == 0 || dist_raw == 0xFFFF) continue;

                double az = std::fmod(az0 + RES_AZ * ch + cfg.angle_offset, 360.0);
                if (az < 0) az += 360.0;
                if (!in_range(az, cfg.a_start, cfg.a_stop)) continue;

                double dist = dist_raw * RES_DIST;
                scan[az] = {dist, rssi};

                if (last_az >= 0 && az < last_az) {   // wrap → full revolution
                    std::cout << "\n=== NEW SCAN ===\n";
                    std::vector<double> keys;
                    keys.reserve(scan.size());
                    for (auto& kv : scan) keys.push_back(kv.first);
                    std::sort(keys.begin(), keys.end(),
                              [inv = cfg.inverted](double a, double b) { return inv ? a > b : a < b; });
                    for (double a : keys) {
                        auto [d, r] = scan[a];
                        std::cout << std::fixed;
                        std::cout.precision(2);
                        std::cout << a << "°  " << d << " m  RSSI=" << int(r) << '\n';
                    }
                    scan.clear();
                }
                last_az = az;
            }
        }
    }
    return 0;
}
