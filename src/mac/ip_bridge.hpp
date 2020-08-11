/*
   Copyright (c) Rice University, RECG Lab
   All rights reserved.
   Licensed under the Apache License, Version 2.0 (the ""License""); you
   may not use this file except in compliance with the License. You may
   obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
   THIS CODE IS PROVIDED ON AN *AS IS* BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION ANY IMPLIED WARRANTIES OR CONDITIONS OF TITLE, FITNESS FOR
   A PARTICULAR PURPOSE, MERCHANTABLITY OR NON-INFRINGEMENT.
   See the Apache Version 2.0 License for specific language governing
   permissions and limitations under the License.
*/

#ifndef IPBRIDGE_H
#define IPBRIDGE_H

class IPbridge {
public:
    IPbridge();
    ~IPbridge();

    void bridge_init(char const* dev_name);
    int tuntap_alloc(char* dev, int flags);
    int write_fragment(unsigned char* buf, int size);
    int read_fragment(unsigned char* buf, int size);

    // Parameters
    //const long maxInSize = 2048;
    const int radioMTU = 1500; // max bytes in one radio frame

private:
    //const int RADIO_HEADER_LENGTH = 2;
    //const int ETHERNET_HEADER_LENGTH = 14; // 6 DstMAC, 6 SrcMAC, 2 EtherType/length
    //const int ETHERNET_MTU = 1600;         // Ethernet frame plus our header
    //const int RADIO_MTU = 1500;            // max bytes in one radio frame
    //const bool USE_RADIO = true;

    int tuntap_fd;
};

#endif
