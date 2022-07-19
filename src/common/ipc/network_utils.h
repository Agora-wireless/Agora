/**
 * @file network_utils.h
 * @brief Helper definations for networking
 */
#ifndef NETWORK_UTILS_H_
#define NETWORK_UTILS_H_

#include <string>

namespace agora_comm {
int ListLocalInterfaces();
//Returns a local interface address in presentation format
std::string GetLocalAddressFromScope(size_t scope_id);
}  // namespace agora_comm

#endif  // NETWORK_UTILS_H_