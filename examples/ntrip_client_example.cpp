#include <spdlog/spdlog.h>
#include <lunar/ntrip_client.h>
#include <lunar/ntrip_util.h>
#include <lunar/rtcm_parser.h>



class MyRTCMListener : public lunar::RTCMListener
{
  public:
    ~MyRTCMListener(){}
    virtual void got_rtcm(const uint8_t* buf, const size_t size) override
    {
        std::string_view s((char*)buf, size);
        spdlog::info("Got rtcm msg(size: {})", size);
    }
};

int main(int argc, const char* argv[])
{

    int port = 2101;
    std::string ip = "37.220.132.38";
    std::string user = "lunarrobot";
    std::string passwd = "Lunar2020";
    std::string mount_point = "SGO_PRS3.2";
    double lat = 47.5619491;
    double lon = 19.0558993;

    libntrip::NtripClient ntrip_client(ip, port, user, passwd, mount_point);

    lunar::RTCM rtcm;
    MyRTCMListener my_listener;
    rtcm.registerListener(&my_listener);

    ntrip_client.OnReceived([&rtcm](const char* charbuffer, const int& size) {
        rtcm.read_cb((const uint8_t*)charbuffer, size);
    });

    std::string gga;
    if (libntrip::GetGGAFrameData(lat, lon, 10.0, &gga) == 0)
    {
        spdlog::info("GGA NMEA sentence: {}", gga.c_str());
        ntrip_client.set_gga_buffer(gga);
    }

    ntrip_client.set_location(lat, lon);
    ntrip_client.set_report_interval(10);

    ntrip_client.Run();

    std::this_thread::sleep_for(std::chrono::seconds(20));

    return 0;
}
