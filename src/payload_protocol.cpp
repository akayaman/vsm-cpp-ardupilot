#include "payload_protocol.h"

using namespace Protocol::Modern;
using namespace Protocol::Modern::Onboard;

OnboardMessage::OnboardMessage(MessageId id, uint8_t length)
{
    m_header.id = id;
    m_header.length = length;
    m_payload.assign(m_header.length, 0);
    updateCrc();
}

OnboardMessage::OnboardMessage(const PayloadByteArray &data)
{
    auto p = data.data();
    auto end = data.data() + data.size();

    auto copy = [&](void* dest, size_t size) {
        auto nextP = p + size;
        if (end < nextP) {
            m_valid  = false;
            return false;
        }
        memcpy(dest, p, size);
        p = nextP;
        return true;
    };

    if (!copy(&m_header, m_header.size))
        return;

    m_payload.resize(m_header.length);
    if (!copy(m_payload.data(), m_payload.size()))
        return;

    if (!copy(&m_crc, sizeof(m_crc)))
        return;
}

uint16_t OnboardMessage::computeCrc() const
{
    uint16_t crc = 0xFFFF;
    updateCrc(crc, &m_header, m_header.size);
    updateCrc(crc, m_payload.data(), m_header.length);
    return crc;
}

void OnboardMessage::updateCrc()
{
    m_crc = computeCrc();
}

PayloadByteArray OnboardMessage::toByteArray()
{
    updateCrc();
    PayloadByteArray result(m_header.size + m_header.length + sizeof(m_crc), 0);
    auto p = result.data();
    memcpy(p, &m_header, m_header.size);
    p += m_header.size;
    memcpy(p, m_payload.data(), m_header.length);
    p += m_header.length;
    memcpy(p, &m_crc, sizeof(m_crc));
    return result;
}

void OnboardMessage::updateCrc(uint16_t &crc, const void *buffer, size_t length)
{
    uint16_t tmp;
    const uint8_t *u8 = reinterpret_cast<const uint8_t*>(buffer);
    for (size_t i = 0; i < length; ++i) {
        tmp = u8[i] ^ (crc & 0xFF);
        tmp ^= (tmp << 4) & 0xFF;
        crc = ((crc >> 8) & 0xFF) ^ static_cast<uint16_t>((tmp << 8) ^ (tmp << 3) ^ ((tmp >> 4) & 0xFF));
    }
}


using namespace Protocol::Modern::Payload;

PayloadMessage::PayloadMessage(const PayloadByteArray &data)
    : m_header(*reinterpret_cast<const PayloadMessageHeader*>(data.data())),
      m_data(data.begin() + sizeof(PayloadMessageHeader),
             data.end())
{
}

PayloadByteArray PayloadMessage::toByteArray()
{
    PayloadByteArray result;
    append(result, m_header.toByteArray());
    append(result, m_data);
    return result;
}

std::vector<PayloadMessage> PayloadMessage::splitDataBlock(CompletePayload completePayload, size_t maxPayloadMessageSize)
{
    std::vector<PayloadMessage> result;
    const uint16_t maxFragmentSize = static_cast<uint16_t>(maxPayloadMessageSize - sizeof(PayloadMessageHeader));
    const uint16_t fullSize = static_cast<uint16_t>(completePayload.data().size());

    for (uint16_t i = 0; i < fullSize; i += maxFragmentSize) {
        auto begin = completePayload.data().data() + i;
        PayloadByteArray fragment(begin, begin + std::min<int>(maxFragmentSize, fullSize - i));
        PayloadMessageHeader header;
        header.totalLength = fullSize;
        header.identification = completePayload.identification();
        header.fragmentOffset = i;
        header.type = completePayload.type();
        header.payloadId = completePayload.payloadId();
        result.push_back(PayloadMessage(header, fragment));
    }
    return result;
}

PayloadMessage::PayloadMessage(PayloadMessageHeader header, const PayloadByteArray &fragment)
    : m_header(header), m_data(fragment)
{

}

namespace Protocol {
namespace Modern {

template void append<uint8_t>(PayloadByteArray&, const uint8_t&);
template void append<uint16_t>(PayloadByteArray&, const uint16_t&);

bool is_little_endian() {
    constexpr union {
        uint32_t i;
        char c[4];
    } checker = { 0x01020304 };
    return checker.c[0] != 1;
}

namespace Logic {

static const std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";


static inline bool is_base64(BYTE c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_encode(BYTE const* buf, unsigned int bufLen) {
  std::string ret;
  int i = 0;
  int j = 0;
  BYTE char_array_3[3];
  BYTE char_array_4[4];

  while (bufLen--) {
    char_array_3[i++] = *(buf++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';
  }

  return ret;
}

std::vector<BYTE> base64_decode(std::string const& encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  BYTE char_array_4[4], char_array_3[3];
  std::vector<BYTE> ret;

  while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_]; in_++;
    if (i ==4) {
      for (i = 0; i <4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);

      char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++)
          ret.push_back(char_array_3[i]);
      i = 0;
    }
  }

  if (i) {
    for (j = i; j <4; j++)
      char_array_4[j] = 0;

    for (j = 0; j <4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++) ret.push_back(char_array_3[j]);
  }

  return ret;
}

}

}
}
