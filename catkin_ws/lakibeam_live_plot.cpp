// lakibeam_live_plot.cpp
// ----------------------------------------------------------------------------
// Build (example): g++ -std=c++17 -O2 lakibeam_live_plot.cpp -o live_plot \
//                  -I/path/to/matplotlib-cpp -lpython3.10
// Run : ./live_plot --sensorip 192.168.8.2 --port 2368
// ----------------------------------------------------------------------------

#include "matplotlibcpp.h"   // https://github.com/lava/matplotlib-cpp
namespace plt = matplotlibcpp;

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

/* ---------- identical constants and CLI parser as previous file ---------- */
static constexpr std::size_t PKT_SIZE = 1248;
static constexpr std::size_t PKT_HDR  = 42;
static constexpr std::size_t BLOCK_SZ = 100;
static constexpr int BLOCKS          = 12;
static constexpr int CH_PER_BLOCK    = 32;
static constexpr double RES_DIST = 0.01;
static constexpr double RES_AZ   = 0.25;

#pragma pack(push,1)
struct BlockHeader { uint16_t flag, az; };
#pragma pack(pop)

struct CLI {
    std::string hostip = "0.0.0.0", sensorip = "";
    int   port = 2368;  bool inverted = false;
    double offset = 0, a0 = 45, a1 = 315;
} cfg;

CLI parse_cli(int argc, char** argv)
{
    CLI o;
    for (int i = 1; i < argc; ++i) {
        auto s = std::string(argv[i]);
        auto next = [&]{ return std::string(argv[++i]); };
        if (s=="--port")          o.port = std::stoi(next());
        else if (s=="--sensorip") o.sensorip = next();
        else if (s=="--hostip")   o.hostip = next();
        else if (s=="--angle_offset") o.offset = std::stod(next());
        else if (s=="--scan_range_start") o.a0 = std::stod(next());
        else if (s=="--scan_range_stop")  o.a1 = std::stod(next());
        else if (s=="--inverted")         o.inverted = true;
    }
    return o;
}

inline bool in_range(double a, double a0, double a1)
{
    return a0 < a1 ? (a>=a0 && a<=a1) : (a>=a0 || a<=a1);
}

int main(int argc, char** argv)
{
    cfg = parse_cli(argc, argv);

    // UDP bind
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{}; addr.sin_family=AF_INET;
    addr.sin_port=htons(cfg.port); addr.sin_addr.s_addr=inet_addr(cfg.hostip.c_str());
    bind(fd,(sockaddr*)&addr,sizeof(addr)); fcntl(fd,F_SETFL,O_NONBLOCK);

    // matplotlib setup
    plt::ion();                          // interactive mode
    plt::figure_size(600,600);
    plt::title("Lakibeam-1 live");
    plt::ylim(0,18); plt::xlim(-18,18);
    plt::axis("equal"); plt::grid(true);

    std::unordered_map<double,double> scan;
    double last = -1; std::array<uint8_t,PKT_SIZE> buf{};

    while (true)
    {
        sockaddr_in src{}; socklen_t sl=sizeof(src);
        ssize_t n=recvfrom(fd,buf.data(),PKT_SIZE,0,(sockaddr*)&src,&sl);
        if (n<0){ usleep(1e4); continue; }
        if (n!=PKT_SIZE) continue;
        if (!cfg.sensorip.empty() && cfg.sensorip!=inet_ntoa(src.sin_addr)) continue;

        for(int b=0;b<BLOCKS;++b){
            const uint8_t* blk=buf.data()+PKT_HDR+b*BLOCK_SZ;
            const auto* bh=reinterpret_cast<const BlockHeader*>(blk);
            if(bh->flag!=0xFFEE) continue;
            double az0=bh->az/100.0;
            for(int i=0;i<CH_PER_BLOCK;++i){
                const uint8_t* t=blk+4+i*3;
                uint16_t d_raw=t[0]|(t[1]<<8);
                if(!d_raw||d_raw==0xFFFF) continue;
                double az=fmod(az0+i*RES_AZ+cfg.offset+360.0,360.0);
                if(!in_range(az,cfg.a0,cfg.a1)) continue;
                double dist=d_raw*RES_DIST;
                scan[az]=dist;
                if(last>=0 && az<last){         // wrap => full scan
                    std::vector<double> xs,ys;
                    xs.reserve(scan.size()); ys.reserve(scan.size());
                    std::vector<double> keys;
                    for(auto&kv:scan)keys.push_back(kv.first);
                    std::sort(keys.begin(),keys.end(),
                              cfg.inverted?std::greater<>() : std::less<>());
                    for(double a:keys){
                        double r=scan[a], rad=a*M_PI/180.0;
                        xs.push_back(r* cos(rad));
                        ys.push_back(r* sin(rad));
                    }
                    plt::clf();
                    plt::scatter(xs,ys,2.0);
                    plt::title("Lakibeam-1 live");
                    plt::xlim(-18,18); plt::ylim(-18,18);
                    plt::axis("equal"); plt::grid(true);
                    plt::pause(0.001);
                    scan.clear();
                }
                last=az;
            }
        }
    }
    return 0;
}
