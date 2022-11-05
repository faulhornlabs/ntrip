#include <spdlog/spdlog.h>
#include <lunar/ntrip_client.h>
#include <lunar/ntrip_util.h>
#include <lunar/bittools.hpp>
#include <lunar/crc.h>


class RTCMListener
{
public:
    virtual void got_rtcm(const uint8_t* buf, const size_t size) = 0;
    virtual ~RTCMListener() {}
};

class RTCM
{
    static constexpr int BUFFER_SIZE = 1024;

public:
    RTCM();

    // feed a byte to the parser
    void read_cb(const uint8_t* buf, size_t size);

    // Registers a new listener to be informed of new RTCM data
    void registerListener(RTCMListener* cb);

    // Restarts the RTCM parsing state (should not be something you find yourself doing a lot)
    void restart();

    // Returns true if the parser has found the start sequence and is in the middle of parsing a new
    // message
    bool parsing_message() const;

    // Returns true if a new message has been parsed.  Subsequent calls to `new_data` will return
    // false until a new messages has been completed
    bool new_data();

private:
    // Decodes the RTCM message
    void decode();

    // Checks that the CRC is correct
    bool check_crc();

    // Working Memory Variables
    uint8_t in_buffer_[BUFFER_SIZE];
    volatile bool new_data_;
    size_t message_len_;  // length, including header and footer
    size_t buffer_head_;
    size_t payload_len_;
    bool start_message_;
    bool end_message_;
    uint8_t ck_a_, ck_b_, ck_c_;
    uint8_t prev_byte_;
    size_t num_errors_;

    enum
    {
        START_BYTE = 0xD3
    };

    enum class ParseState
    {
        START,
        GOT_START_FRAME,
        GOT_CLASS,
        GOT_MSG_ID,
        GOT_LENGTH1,
        GOT_LENGTH2,
        GOT_PAYLOAD,
        GOT_CK_A,
        GOT_CK_B,
        GOT_CK_C,
        DONE,
    };
    ParseState parse_state_;

    std::vector<RTCMListener*> listeners_;
};

class MyRTCMListener : public RTCMListener
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

    RTCM rtcm;
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


RTCM::RTCM()
{
    buffer_head_ = 0;
    parse_state_ = ParseState::START;
    prev_byte_ = 0;

    start_message_ = false;
    new_data_ = false;
    end_message_ = false;
}

void RTCM::restart()
{
    parse_state_ = ParseState::START;
    start_message_ = false;
    end_message_ = false;
}

bool RTCM::parsing_message() const
{
    return (start_message_ == true && end_message_ == false);
}

bool RTCM::new_data()
{
    bool tmp = new_data_;
    new_data_ = false;
    return tmp;
}

bool RTCM::check_crc()
{
    uint32_t read_crc = getBit<24>(in_buffer_, (buffer_head_ - 3) * 8);
    uint32_t computed_crc = crc24(in_buffer_, payload_len_ + 3);
    return read_crc == computed_crc;
}

void RTCM::read_cb(const uint8_t* buf, size_t size)
{
    for (int i = 0; i < size; ++i)
    {
        uint8_t byte = buf[i];
        switch (parse_state_)
        {
        case ParseState::START:
            buffer_head_ = 0;
            if (byte == START_BYTE)
            {
                parse_state_ = ParseState::GOT_START_FRAME;
                payload_len_ = 0;
                ck_a_ = 0;
                ck_b_ = 0;
                ck_c_ = 0;
                start_message_ = true;
                end_message_ = false;
                in_buffer_[buffer_head_++] = byte;
            }
            break;
        case ParseState::GOT_START_FRAME:
            in_buffer_[buffer_head_++] = byte;
            parse_state_ = ParseState::GOT_LENGTH1;
            break;
        case ParseState::GOT_LENGTH1:
            in_buffer_[buffer_head_++] = byte;
            payload_len_ = ((prev_byte_ & 0x3) << 8) | byte;
            parse_state_ = ParseState::GOT_LENGTH2;
            if (payload_len_ > BUFFER_SIZE || payload_len_ == 0)
            {
                num_errors_++;
                prev_byte_ = byte;
                restart();
            }
            break;
        case ParseState::GOT_LENGTH2:
            in_buffer_[buffer_head_++] = byte;
            if (buffer_head_ - 3 == payload_len_)  // remember that buffer_head includes the header
            {
                parse_state_ = ParseState::GOT_PAYLOAD;
            }
            if (buffer_head_ - 3 > payload_len_)  // remember that buffer_head includes the header
            {
                spdlog::error("buffer head {} > payload length {}", buffer_head_ - 3 , payload_len_);
                fflush(stdout);
                num_errors_++;
                parse_state_ = ParseState::START;
                prev_byte_ = byte;
                start_message_ = false;
                end_message_ = false;
            }
            break;
        case ParseState::GOT_PAYLOAD:
            in_buffer_[buffer_head_++] = byte;
            ck_a_ = byte;
            parse_state_ = ParseState::GOT_CK_A;
            break;
        case ParseState::GOT_CK_A:
            in_buffer_[buffer_head_++] = byte;
            ck_b_ = byte;
            parse_state_ = ParseState::GOT_CK_B;
            break;
        case ParseState::GOT_CK_B:
            in_buffer_[buffer_head_++] = byte;
            ck_c_ = byte;
            parse_state_ = ParseState::GOT_CK_C;
            if (buffer_head_ > BUFFER_SIZE)
            {
                buffer_head_ = 0;
                num_errors_++;
                parse_state_ = ParseState::START;
                start_message_ = false;
                end_message_ = true;
            }
            break;
        default:
            buffer_head_ = 0;
            num_errors_++;
            parse_state_ = ParseState::START;
            start_message_ = false;
            end_message_ = true;
            break;
        }

        // If we have a complete packet, then try to parse it
        if (parse_state_ == ParseState::GOT_CK_C)
        {
            parse_state_ = ParseState::START;
            start_message_ = false;
            end_message_ = true;
            decode();
            prev_byte_ = byte;
        }

        prev_byte_ = byte;
    }
}

void RTCM::decode()
{
    new_data_ = true;
    message_len_ = buffer_head_;
    if (check_crc())
    {
        for (auto& l : listeners_)
        {
            l->got_rtcm(in_buffer_, buffer_head_);
        }
    }
    else
    {
        spdlog::error("RTCM not pass CRC check");
    }
}

void RTCM::registerListener(RTCMListener* l)
{
    listeners_.push_back(l);
}