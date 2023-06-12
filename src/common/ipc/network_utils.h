/**
 * @file network_utils.h
 * @brief Helper definations for networking
 */
#ifndef NETWORK_UTILS_H_
#define NETWORK_UTILS_H_

///For addr info
#include <netdb.h>

#include <string>

namespace agora_comm {
int ListLocalInterfaces();
//Returns a local interface address in presentation format
std::string GetLocalAddressFromScope(size_t scope_id);
::addrinfo* GetAddressInfo(const std::string& local_address,
                           const std::string& local_port);
void PrintAddressInfo(const ::addrinfo* print_info);
}  // namespace agora_comm

#endif  // NETWORK_UTILS_H_