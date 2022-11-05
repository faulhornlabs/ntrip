
#ifndef NTRIPLIB_NTRIP_CLIENT_H_
#define NTRIPLIB_NTRIP_CLIENT_H_

#include <atomic>
#include <string>
#include <thread>  // NOLINT.
#include <functional>
#include <boost/asio.hpp>

namespace libntrip {

using ClientCallback = std::function<void(const char *, const int &)>;

class NtripClient {
 public:
  NtripClient() = default;
  NtripClient(const NtripClient &) = delete;
  NtripClient& operator=(const NtripClient &) = delete;
  NtripClient(const std::string &ip, const int &port,
              const std::string &user, const std::string &passwd,
              const std::string &mountpoint) :
      server_ip_(ip),
      server_port_(port),
      user_(user),
      passwd_(passwd),
      mountpoint_(mountpoint) { }
  ~NtripClient() { Stop(); }

  void Init(const std::string &ip, const int &port,
            const std::string &user, const std::string &passwd,
            const std::string &mountpoint) {
    server_ip_ = ip;
    server_port_ = port;
    user_ = user;
    passwd_ = passwd;
    mountpoint_ = mountpoint;
    service_is_running_ = false;
  }
  // 更新发送的GGA语句.
  // 根据ntrip账号的要求, 如果距离服务器位置过远, 服务器不会返回差分数据.
  void set_gga_buffer(const std::string &gga_buffer) {
    gga_buffer_ = gga_buffer;
  }
  // 设置固定位置坐标, 由此自动生成GGA数据.
  void set_location(double const& latitude, double const& longitude) {
    latitude_ = latitude;
    longitude_ = longitude;
  }
  // 设置GGA上报时间间隔, 单位秒(s).
  void set_report_interval(int const& intv) {
    report_interval_ = intv;
  }

  // 设置接收到数据时的回调函数.
  void OnReceived(const ClientCallback &callback) { callback_ = callback; }
  bool Run(void);
  void Stop(void);
  bool service_is_running(void) const {
    return service_is_running_;
  }

 private:
  // Thread handler.
  void TheradHandler(void);
  void handleRead(const boost::system::error_code& error, size_t bytes_transferred);

  std::atomic_bool service_is_running_;
  std::atomic_bool gga_is_update_;  // 外部更新GGA数据标志.
  int report_interval_;  // GGA数据上报时间间隔.
  double latitude_ = 22.570535;  // 固定坐标纬度.
  double longitude_ = 113.937739;  // 固定坐标经度.
  std::thread thread_;
  std::string server_ip_;
  int server_port_ = -1;
  std::string user_;
  std::string passwd_;
  std::string mountpoint_;
  std::string gga_buffer_;
  ClientCallback callback_;

  boost::asio::io_service m_io_service;
  std::unique_ptr<boost::asio::io_service::work> m_io_work;
  std::unique_ptr<boost::asio::ip::tcp::socket> m_socket;
  std::array<char, 2048> m_recv_buf;
  std::chrono::steady_clock::time_point m_start_tp;

};

}  // namespace libntrip

#endif  // NTRIPLIB_NTRIP_CLIENT_H_
