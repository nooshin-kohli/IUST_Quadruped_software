/**
 * @file
 * This file implements functions to receive
 * and transmit CAN frames via SocketCAN.
 */

#include <string>
#ifndef MINGW

#include <stdio.h>
// strncpy
#include <string.h>
// close
#include <unistd.h>
// socket
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

#include "rt/SocketCAN.h"


SocketCAN::SocketCAN(std::function<void(can_frame_t*)> reception_handler_f)
:reception_handler{std::move(reception_handler_f)},sockfd(-1), receiver_thread_id(0) {
    printf("SocketCAN adapter created.\n");
}


SocketCAN::~SocketCAN() {
    printf("Destroying SocketCAN adapter...\n");
    if (this->is_open()) {
        this->close();
    }
}


void SocketCAN::open(const char* interface) {

    // Setting can interface
    std::string can_i(interface);
    __can_conf_1 = "sudo ip link set " + can_i + " type can bitrate 1000000";
    __can_conf_2 = "sudo ifconfig " + can_i + " txqueuelen 65536";
    __can_conf_3 = "sudo ifconfig " + can_i + " up";
    __can_conf_4 = "sudo ifconfig " + can_i + " down";



    int systemRet;
    systemRet = system(this->__can_conf_4.c_str());
    if (systemRet == -1) { // TODO: not catching errors
	printf("can down command failed");
    }
    systemRet = system(this->__can_conf_1.c_str());
    if (systemRet == -1) {
	printf("can bitrate command failed");
    }
    systemRet = system(this->__can_conf_2.c_str());
    if (systemRet == -1) {
	printf("can queue command failed");
    }
    systemRet = system(this->__can_conf_3.c_str());
    if (systemRet == -1) {
	printf("can up command failed");
    }

    // Request a socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd == -1) {
        printf("Error: Unable to create a CAN socket\n");
        return;
    }
    printf("Created CAN socket with descriptor %d.\n\n\n", sockfd);
int size = 8*26624;
setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
    // Get the index of the network interface
    strncpy(if_request.ifr_name, interface, IFNAMSIZ);
    if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1) {
        printf("Unable to select CAN interface %s: I/O control error\n", interface);

        // Invalidate unusable socket
        close();
        return;
    }
    printf("Found: %s has interface index %d.\n", interface, if_request.ifr_ifindex);

    // Bind the socket to the network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_request.ifr_ifindex;
    int rc = bind(
        sockfd,
        reinterpret_cast<struct sockaddr*>(&addr),
        sizeof(addr)
    );
    if (rc == -1) {
        printf("Failed to bind socket to network interface\n");

        // Invalidate unusable socket
        close();
        return;
    }
    printf("Successfully bound socket to interface %d.\n", if_request.ifr_ifindex);

    // Start a separate, event-driven thread for frame reception
    start_receiver_thread();
}

void SocketCAN::close() {
    terminate_receiver_thread = true;

    if (!is_open())
        return;

    // Close the file descriptor for our socket
    ::close(sockfd);
    sockfd = -1;

    printf("CAN socket destroyed.\n");
}

bool SocketCAN::is_open() {
    return (sockfd != -1);
}

void SocketCAN::transmit(can_frame_t* can_frame) {
    if (!is_open()) {
        printf("Unable to transmit: Socket not open\n");
        return;
    }
    
    int nbytes = write(this->sockfd, can_frame, sizeof(can_frame_t));

    if (nbytes != sizeof(*can_frame)) {
        printf("Transmit Error!\r\n");
    }
}


static void* socketcan_receiver_thread(void* argv) {
    /*
     * The first and only argument to this function
     * is the pointer to the object, which started the thread.
     */
    SocketCAN* sock = (SocketCAN*) argv;

    // Holds the set of descriptors, that 'select' shall monitor
    fd_set descriptors;

    // Highest file descriptor in set
    int maxfd = sock->sockfd;

    // How long 'select' shall wait before returning with timeout
    struct timeval timeout;

    // Buffer to store incoming frame
    can_frame_t rx_frame;

    // Run until termination signal received
    while (!sock->terminate_receiver_thread) {
        // Clear descriptor set
        FD_ZERO(&descriptors);
        // Add socket descriptor
        FD_SET(sock->sockfd, &descriptors);
    //    printf("Added %d to monitored descriptors.\n", sock->sockfd);

        // Set timeout
        timeout.tv_sec  = 0;
        timeout.tv_usec = 100;

        // Wait until timeout or activity on any descriptor
        if (select(maxfd+1, &descriptors, NULL, NULL, &timeout) == 1) {
            int len = read(sock->sockfd, &rx_frame, CAN_MTU);
        //    printf("Received %d bytes: Frame from 0x%0X, DLC=%d\n", len, rx_frame.can_id, rx_frame.can_dlc);

            if (len < 0)
                continue;

            // callback function
            if (sock->reception_handler != NULL) {
                sock->reception_handler(&rx_frame);
            }
        }
        else {
//            printf("Received nothing.\n");
        }
    }

    printf("Receiver thread terminated.\n");

    // Thread terminates
    return NULL;
}


void SocketCAN::start_receiver_thread() {
    /*
     * Frame reception is accomplished in a separate, event-driven thread.
     *
     * See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
     */
    terminate_receiver_thread = false;
    int rc = pthread_create(&receiver_thread_id, NULL, &socketcan_receiver_thread, this);
    if (rc != 0) {
        printf("Unable to start receiver thread.\n");
        return;
    }
    printf("Successfully started receiver thread with ID %d.\n", (int) receiver_thread_id);
}

#endif
