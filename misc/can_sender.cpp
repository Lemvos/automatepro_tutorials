#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

bool running = true;

void signal_handler(int) {
    running = false;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./can_sender <interface>\nExample: ./can_sender vcan0\n";
        return 1;
    }

    signal(SIGINT, signal_handler);

    std::string ifname = argv[1];

    // Open socket
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Sender socket");
        return 1;
    }

    // Enable loopback in case of vcan
    int loopback = 1;
    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // Bind socket
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        return 1;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    std::cout << "[Sender] Sending on interface: " << ifname << ". Press Ctrl+C to stop.\n";

    int counter = 0;
    while (running) {
        struct can_frame frame {};
        frame.can_id = 0x123;
        frame.can_dlc = 2;
        frame.data[0] = counter & 0xFF;
        frame.data[1] = (counter >> 8) & 0xFF;

        int bytes_sent = write(sock, &frame, sizeof(frame));
        if (bytes_sent == sizeof(frame)) {
            std::cout << "[Sender] Sent ID 0x" << std::hex << frame.can_id
                      << " Data: " << std::dec << static_cast<int>(frame.data[0])
                      << " " << static_cast<int>(frame.data[1]) << std::endl;
        } else {
            perror("[Sender] Write error");
        }

        ++counter;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(sock);
    std::cout << "[Sender] Closed.\n";
    return 0;
}
