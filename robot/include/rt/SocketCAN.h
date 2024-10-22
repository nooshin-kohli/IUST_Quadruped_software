/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#include <functional>
#if (!defined(SOCKETCAN_H)) && (!defined(MINGW))
#define SOCKETCAN_H

#include <linux/can.h>
#include <stdbool.h>
// IFNAMSIZ, ifreq
#include <net/if.h>
// Multi-threading
#include <pthread.h>
#include <string>

// Workaround for absent linux headers: Explicit struct definition
/* struct can_frame
{
    uint32_t    can_id;
    uint8_t     can_dlc;
    uint8_t     __pad;
    uint8_t     __res0;
    uint8_t     __res1;
    uint8_t     data[8] __attribute__((aligned(8)));
}; */

/**
 * Holds the content of one CAN frame
 * Struct delcared in <linux/can.h>
 */
typedef struct can_frame can_frame_t;

/**
 * Interface request structure used for socket ioctl's
 */
typedef struct ifreq interface_request_t;

/**
 * Socket address type for CAN sockets
 */
typedef struct sockaddr_can can_socket_address_t;


/**
 * Facilitates frame transmission and reception via a CAN adapter
 */
class SocketCAN {
  private:
    std::string __can_conf_1;
    std::string __can_conf_2;
    std::string __can_conf_3;
    std::string __can_conf_4;

    interface_request_t if_request;

    
  public:
    /** Constructor */
    SocketCAN(std::function<void(can_frame_t*)>);
    /** Destructor */
    ~SocketCAN();

can_socket_address_t addr;
    /**
     * Pointer to a function which shall be called
     * when frames are being received from the CAN bus
     */
    std::function<void(can_frame_t*)> reception_handler;
    
	/**
     * CAN socket file descriptor
     */
    int sockfd;
	
    pthread_t receiver_thread_id;


    /**
     * Request for the child thread to terminate
     */
    bool terminate_receiver_thread = false;

    /**
     * Open and bind socket
     */
    void open(const char*);

    /**
     * Close and unbind socket
     */
    void close();

    /**
     * Returns whether the socket is open or closed
     *
     * @retval true     Socket is open
     * @retval false    Socket is closed
     */
    bool is_open();

    /**
     * Sends the referenced frame to the bus
     */
    void transmit(can_frame_t*);

    /**
     * Starts a new thread, that will wait for socket events
     */
    void start_receiver_thread();
};

#endif
