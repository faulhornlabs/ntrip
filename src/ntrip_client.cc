// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ntrip/ntrip_client.h"

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <string>
#include <list>
#include <vector>

#include "ntrip/ntrip_util.h"
#include <iostream>


namespace libntrip {

namespace {

// GPGGA format example.
constexpr char gpgga_buffer[] =
    "$GPGGA,083552.00,3000.0000000,N,11900.0000000,E,"
    "1,08,1.0,0.000,M,100.000,M,,*57\r\n";

}  // namespace

//
// Public method.
//

bool NtripClient::Run(void) {
  Stop();
  service_is_running_.store(false);
  int ret = -1;
  //char recv_buf[1024] = {0};
  char request_data[1024] = {0};
  char userinfo_raw[48] = {0};
  char userinfo[64] = {0};
  // Generate base64 encoding of username and password.
  snprintf(userinfo_raw, sizeof(userinfo_raw) , "%s:%s",
           user_.c_str(), passwd_.c_str());
  Base64Encode(userinfo_raw, userinfo);
  // Generate request data format of ntrip.
  snprintf(request_data, sizeof(request_data),
           "GET /%s HTTP/1.1\r\n"
           "User-Agent: %s\r\n"
           "Authorization: Basic %s\r\n"
           "\r\n",
           mountpoint_.c_str(), kClientAgent, userinfo);


  try
  {
      m_io_service.restart();
      m_io_work = std::make_unique<boost::asio::io_service::work>(m_io_service);
      boost::asio::ip::tcp::endpoint ep(
          boost::asio::ip::address::from_string(server_ip_),
          server_port_);

      m_socket = std::make_unique<boost::asio::ip::tcp::socket>(m_io_service);
      m_socket->connect(ep);

      thread_ = std::thread([this]() {m_io_service.run(); });
  }
  catch (const std::exception& e)
  {
      std::cerr << e.what() << std::endl;
      return false;
  }
  boost::asio::const_buffer data_to_write(request_data, strlen(request_data));
  auto data_written_size = m_socket->write_some(data_to_write);
  // Send request data.
  if (data_written_size == 0) {
    std::cerr << "Send request failed!!!\n";
    return false;
  }
  // Waitting for request to connect caster success.
  int timeout = 3;
  boost::system::error_code e_code;
  while (timeout--) {
    //ret = recv(socket_fd, recv_buf, sizeof(recv_buf), 0);
    boost::asio::mutable_buffer b(m_recv_buf.data(), m_recv_buf.size());
    ret = m_socket->read_some(b, e_code);
    if (ret == 0)
    {
        e_code.message();
    }
    if (ret > 0) {
      std::string result(m_recv_buf.data(), ret);
      std::cout << result << std::endl;

      if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
          (result.find("ICY 200 OK") != std::string::npos)) 
      {
        if (gga_buffer_.empty()) {
          GetGGAFrameData(latitude_, longitude_, 10.0, &gga_buffer_);
        }
        boost::asio::const_buffer data_to_write(gga_buffer_.c_str(), gga_buffer_.size());
        ret = m_socket->write_some(data_to_write, e_code);
        if (ret == 0) 
        {
          std::cerr << e_code.message() << " Send gpgga data fail\n";
          return false;
        }
        // printf("Send gpgga data ok\n");
        break;
      }
      else
      {
          m_socket->close();
          return false;
      }
    } else if (ret == 0) {
      printf("Remote socket close!!!\n");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (timeout <= 0) {
    return false;
  }

  service_is_running_ = true;
  m_start_tp = std::chrono::steady_clock::now();
  m_socket->async_read_some(boost::asio::buffer(m_recv_buf.data(), m_recv_buf.size()),
      std::bind(&NtripClient::handleRead, this,
          std::placeholders::_1,
          std::placeholders::_2));
  //thread_ = std::thread(&NtripClient::TheradHandler, this);
  //thread_.detach();
  printf("NtripClient service starting ...\n");
  gga_is_update_.store(false);
  return true;
}

void NtripClient::Stop(void) {
  service_is_running_.store(false);
  m_io_work.reset();
  m_io_service.stop();
  if (thread_.joinable())
      thread_.join();
}

//
// Private method.
//

//void NtripClient::TheradHandler(void) {
//  service_is_running_.store(true);
//  int ret;
//  char recv_buffer[1024] = {};
//  auto start_tp = std::chrono::steady_clock::now();
//  int intv_ms = report_interval_ * 1000;
//  while (service_is_running_) {
//    ret = recv(socket_fd_, recv_buffer, sizeof(recv_buffer), 0);
//    if (ret == 0) {
//      printf("Remote socket close!!!\n");
//      break;
//    } else if (ret < 0) {
//      if ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR)) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//      } else {
//        printf("Remote socket error!!!\n");
//        break;
//      }
//    } else {
//      callback_(recv_buffer, ret);
//    }
//    if (std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::steady_clock::now()-start_tp).count() >= intv_ms) {
//      start_tp = std::chrono::steady_clock::now();
//      if (gga_is_update_ == false) {
//        GetGGAFrameData(latitude_, longitude_, 10.0, &gga_buffer_);
//      }
//      send(socket_fd_, gga_buffer_.c_str(), gga_buffer_.size(), 0);
//      gga_is_update_.store(false);
//    }
//  }
//  close(socket_fd_);
//  socket_fd_ = -1;
//  service_is_running_ = false;
//}

void NtripClient::handleRead(const boost::system::error_code& error, size_t bytes_transferred)
{

    if (error != boost::system::errc::success)
    {
        service_is_running_ = false;
        std::cout << error.message() << std::endl;
    }

    callback_(m_recv_buf.data(), bytes_transferred);

    int intv_ms = report_interval_ * 1000;


    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - m_start_tp).count() >= intv_ms) {
        m_start_tp = std::chrono::steady_clock::now();
        if (gga_is_update_ == false) {
            GetGGAFrameData(latitude_, longitude_, 10.0, &gga_buffer_);
        }
        boost::asio::const_buffer data_to_write(gga_buffer_.c_str(), gga_buffer_.size());
        boost::system::error_code e_code;
        auto ret = m_socket->write_some(data_to_write, e_code);
        if (ret == 0)
        {
            std::cerr << e_code.message() << " Send gpgga data fail\n";
        }
        //send(socket_fd_, gga_buffer_.c_str(), gga_buffer_.size(), 0);
        gga_is_update_.store(false);
    }

    m_socket->async_read_some(boost::asio::buffer(m_recv_buf.data(), m_recv_buf.size()),
        std::bind(&NtripClient::handleRead, this,
            std::placeholders::_1,
            std::placeholders::_2));

}

}  // namespace libntrip
