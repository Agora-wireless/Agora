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



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
//#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <stdarg.h>

#include "ip_bridge.hpp"


IPbridge::IPbridge() 
{
    tuntap_fd = -1;
    char* dev_name = "tun44";
    bridge_init(dev_name);
}

IPbridge::~IPbridge()
{

}


int IPbridge::tuntap_alloc(char *dev, int flags) {
    /*
     *
     * Create virtual interface
     * Input: char *dev - name of virtual device (if '\0', kernel provides name)
     *        flags - interface flags, options such as whether TUN or TAP
     * Output: fd - File descriptor
     *
     */

    // Interface request
    struct ifreq ifr;
    // File Descriptor, Error
    int fd, err;
    // Tun device
    char const *clonedev = "/dev/net/tun";

    if( (fd = open(clonedev , O_RDWR)) < 0 ) {
        perror("Opening /dev/net/tun");
        return fd;
    }

    // Zero init ifreq struct
    memset(&ifr, 0, sizeof(ifr));

    // IFF_TUN or IFF_TAP, plus maybe IFF_NO_PI
    ifr.ifr_flags = flags;

    // If device passed to tun_alloc add it to ifreq
    if (*dev) {
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);
    }

    /* Ask kernel to create new device via ioctl() system call. 
     * Pass file descriptor, TUNSETIFF constant, pointer to data structure with
     * interface name and operation mode. */ 
    if( (err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0 ) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return err;
    }

    /* If successful, write device name to dev variable. File descriptor the 
     * caller will use to talk to virtual inteface */
    strcpy(dev, ifr.ifr_name);
    printf("TUN IFACE ALLOC SUCCESSFUL \n");
    return fd;
}


void IPbridge::bridge_init(char const * dev_name)
{
    char tuntap_name[IFNAMSIZ];

    /* Connect to the device */
    strcpy(tuntap_name, dev_name);
    if (strstr(dev_name, "tap")) {
        tuntap_fd = tuntap_alloc(tuntap_name, IFF_TAP | IFF_NO_PI);  /* tap interface */
    } else {
        tuntap_fd = tuntap_alloc(tuntap_name, IFF_TUN | IFF_NO_PI);  /* tun interface */
    }
    if(tuntap_fd < 0){
        perror("Allocating TUN/TAP IF");
        exit(1);
    }
}


int IPbridge::write_fragment(unsigned char * buf, int size)
{
    // Write to device - Upstream

    int nwrite;

    if((nwrite=write(tuntap_fd, buf, size)) < 0){
        perror("Writing data to TUN/TAP IF");
        exit(1);
    }
    return nwrite;
}


int IPbridge::read_fragment(unsigned char * buf, int size)
{
    // Read from device - Downstream
    /* Note that "buffer" should be at least the MTU size of the interface, eg 1500 bytes */
    int nread;
    nread=read(tuntap_fd, buf, size);
    if(nread < 0){
        perror("Reading data from TUN/TAP Interface");
        exit(1);
    }
    return nread;
}
