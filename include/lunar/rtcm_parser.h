#pragma once
#include <vector>
#include <inttypes.h>

namespace lunar
{

class RTCMListener
{
  public:
    virtual void got_rtcm(const uint8_t* buf, const size_t size) = 0;
    virtual ~RTCMListener()
    {
    }
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
    size_t message_len_; // length, including header and footer
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
} // namespace lunar