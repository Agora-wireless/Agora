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

/*
 *  this code writes/reads to/from a tun virtual interface and therefore creates
 *  an actual interface for users to send and receive packets over the air
 *  the following tutorial was very helpful in writing this: 
 *  http://backreference.org/2010/03/26/tuntap-interface-tutorial/ and
 *  http://thgeorgiou.com/posts/2017-03-20-usb-serial-network/
 */

#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
//#include <linux/if.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include "ip_bridge.hpp"

IPbridge::IPbridge()
{
    tuntap_fd = -1;
    char* dev_name = "tun44";
    bridge_init(dev_name);
}

IPbridge::~IPbridge() {}

int IPbridge::tuntap_alloc(char* dev, int flags)
{
    /*
     *
     * Create virtual interface
     * Input: char *dev - name of virtual device (if '\0', kernel provides name)
     *        flags - interface flags, options such as whether IFF_TUN or IFF_TAP
     */

    // Interface request
    struct ifreq ifr;
    int fd, err;
    char const* clonedev = "/dev/net/tun";

    if ((fd = open(clonedev, O_RDWR)) < 0) {
        perror("Opening /dev/net/tun");
        return fd;
    }

    memset(&ifr, 0, sizeof(ifr));

    ifr.ifr_flags = flags;

    if (*dev) {
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);
    }

    if ((err = ioctl(fd, TUNSETIFF, (void*)&ifr)) < 0) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return err;
    }

    strcpy(dev, ifr.ifr_name);
    printf("TUN IFACE ALLOC SUCCESSFUL \n");
    return fd;
}

void IPbridge::bridge_init(char const* dev_name)
{
    char tuntap_name[IFNAMSIZ];

    /* Connect to the device */
    strcpy(tuntap_name, dev_name);
    if (strstr(dev_name, "tap")) {
        tuntap_fd = tuntap_alloc(tuntap_name, IFF_TAP | IFF_NO_PI);
    } else {
        tuntap_fd = tuntap_alloc(tuntap_name, IFF_TUN | IFF_NO_PI);
    }
    if (tuntap_fd < 0) {
        perror("Allocating TUN/TAP IF");
        exit(1);
    }
}

int IPbridge::write_fragment(unsigned char* buf, int size)
{
    // Write to device - Upstream

    int nwrite;

    if ((nwrite = write(tuntap_fd, buf, size)) < 0) {
        perror("Writing data to TUN/TAP IF");
        exit(1);
    }
    return nwrite;
}

int IPbridge::read_fragment(unsigned char* buf, int size)
{
    // Read from device - Downstream
    /* Note that "buffer" should be at least the MTU size of the interface, eg 1500 bytes */
    int nread;
    nread = read(tuntap_fd, buf, size);
    if (nread < 0) {
        perror("Reading data from TUN/TAP Interface");
        exit(1);
    }
    return nread;
}
