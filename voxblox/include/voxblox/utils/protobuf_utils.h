#ifndef VOXBLOX_UTILS_PROTOBUF_UTILS_H_
#define VOXBLOX_UTILS_PROTOBUF_UTILS_H_

#include <fstream>

#include <glog/logging.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>

namespace voxblox {

namespace utils {
bool readProtoMsgCountToStream(std::fstream* stream_in, uint32_t* message_count,
                               uint64_t* byte_offset);

bool writeProtoMsgCountToStream(uint32_t message_count,
                                std::fstream* stream_out);

bool readProtoMsgFromStream(std::fstream* stream_in,
                            google::protobuf::Message* message,
                            uint64_t* byte_offset);

bool writeProtoMsgToStream(const google::protobuf::Message& message,
                           std::fstream* stream_out);

}  // namespace utils
}  // namespace voxblox

#endif  // VOXBLOX_UTILS_PROTOBUF_UTILS_H_
