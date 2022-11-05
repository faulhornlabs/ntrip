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

#include <lunar/ntrip_client.h>
#include <lunar/ntrip_util.h>

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <string>
#include <list>
#include <vector>

#include <spdlog/spdlog.h>

namespace libntrip
{

namespace
{

// GPGGA format example.
constexpr char gpgga_buffer[] = "$GPGGA,083552.00,3000.0000000,N,11900.0000000,E,"
                                "1,08,1.0,0.000,M,100.000,M,,*57\r\n";

} // namespace

//
// Public method.
//

bool NtripClient::Run(void)
{
    Stop();
    service_is_running_.store(false);
    int ret = -1;
    // char recv_buf[1024] = {0};
    char request_data[1024] = {0};
    char userinfo_raw[48] = {0};
    char userinfo[64] = {0};
    // Generate base64 encoding of username and password.
    snprintf(userinfo_raw, sizeof(userinfo_raw), "%s:%s", user_.c_str(), passwd_.c_str());
    Base64Encode(userinfo_raw, userinfo);
    // Generate request data format of ntrip.
    snprintf(request_data,
        sizeof(request_data),
        "GET /%s HTTP/1.1\r\n"
        "User-Agent: %s\r\n"
        "Authorization: Basic %s\r\n"
        "\r\n",
        mountpoint_.c_str(),
        kClientAgent,
        userinfo);

    try
    {
        m_io_service.restart();
        m_io_work = std::make_unique<boost::asio::io_service::work>(m_io_service);
        boost::asio::ip::tcp::endpoint ep(
            boost::asio::ip::address::from_string(server_ip_), server_port_);

        m_socket = std::make_unique<boost::asio::ip::tcp::socket>(m_io_service);
        m_socket->connect(ep);

        thread_ = std::thread([this]() { m_io_service.run(); });
    }
    catch (const std::exception& e)
    {
        spdlog::error("Got exception: {}", e.what());
        return false;
    }
    boost::asio::const_buffer data_to_write(request_data, strlen(request_data));
    auto data_written_size = m_socket->write_some(data_to_write);
    // Send request data.
    if (data_written_size == 0)
    {
        spdlog::error("Send request failed!!!");
        return false;
    }
    // Waitting for request to connect caster success.
    int timeout = 3;
    boost::system::error_code e_code;
    while (timeout--)
    {
        boost::asio::mutable_buffer b(m_recv_buf.data(), m_recv_buf.size());
        ret = m_socket->read_some(b, e_code);
        if (ret == 0)
        {
            e_code.message();
        }
        if (ret > 0)
        {
            std::string result(m_recv_buf.data(), ret);
            spdlog::info("Received: {}", result);

            if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
                (result.find("ICY 200 OK") != std::string::npos))
            {
                if (gga_buffer_.empty())
                {
                    GetGGAFrameData(latitude_, longitude_, 10.0, &gga_buffer_);
                }
                boost::asio::const_buffer data_to_write(gga_buffer_.c_str(), gga_buffer_.size());
                ret = m_socket->write_some(data_to_write, e_code);
                if (ret == 0)
                {
                    spdlog::error("Send gpgga data fail: {}", e_code.message());
                    return false;
                }
                break;
            }
            else
            {
                m_socket->close();
                return false;
            }
        }
        else if (ret == 0)
        {
            spdlog::error("Remote socket closed connection!!!");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (timeout <= 0)
    {
        return false;
    }

    service_is_running_ = true;
    m_start_tp = std::chrono::steady_clock::now();
    m_socket->async_read_some(boost::asio::buffer(m_recv_buf.data(), m_recv_buf.size()),
        std::bind(&NtripClient::handleRead, this, std::placeholders::_1, std::placeholders::_2));
    spdlog::info("NtripClient service starting ...");
    gga_is_update_.store(false);
    return true;
}

void NtripClient::Stop(void)
{
    service_is_running_.store(false);
    m_io_work.reset();
    m_io_service.stop();
    if (thread_.joinable())
        thread_.join();
}

void NtripClient::handleRead(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (error != boost::system::errc::success)
    {
        service_is_running_ = false;
        spdlog::error("Error reading: {}", error.message());
    }

    callback_(m_recv_buf.data(), bytes_transferred);

    int intv_ms = report_interval_ * 1000;

    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - m_start_tp)
            .count() >= intv_ms)
    {
        m_start_tp = std::chrono::steady_clock::now();
        if (gga_is_update_ == false)
        {
            GetGGAFrameData(latitude_, longitude_, 10.0, &gga_buffer_);
        }
        boost::asio::const_buffer data_to_write(gga_buffer_.c_str(), gga_buffer_.size());
        boost::system::error_code e_code;
        auto ret = m_socket->write_some(data_to_write, e_code);
        if (ret == 0)
        {
            spdlog::error(" Send gpgga data fail: {}", e_code.message());
        }
        gga_is_update_.store(false);
    }

    m_socket->async_read_some(boost::asio::buffer(m_recv_buf.data(), m_recv_buf.size()),
        std::bind(&NtripClient::handleRead, this, std::placeholders::_1, std::placeholders::_2));
}

} // namespace libntrip
