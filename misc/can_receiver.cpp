#include <iostream>
#include <cstring>
#include <thread>
#include <csignal>
#include <cstdlib>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <sys/select.h>

bool running = true;

void signal_handler(int) {
    running = false;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./can_receiver <interface>\nExample: ./can_receiver vcan0\n";
        return 1;
    }

    signal(SIGINT, signal_handler);

    std::string ifname = argv[1];

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Receiver socket");
        return 1;
    }

    int loopback = 1;
    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

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

    std::cout << "[Receiver] Listening on interface: " << ifname << ". Press Ctrl+C to stop.\n";

    struct can_frame frame;
    fd_set read_fds;
    struct timeval timeout;

    while (running) {
        FD_ZERO(&read_fds);
        FD_SET(sock, &read_fds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(sock + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(sock, &read_fds)) {
            int nbytes = read(sock, &frame, sizeof(frame));
            if (nbytes > 0) {
                std::cout << "[Receiver] Received ID 0x" << std::hex << frame.can_id << " Data: ";
                for (int i = 0; i < frame.can_dlc; ++i)
                    std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
                std::cout << std::dec << std::endl;
            } else {
                perror("[Receiver] Read error");
            }
        }
    }

    close(sock);
    std::cout << "[Receiver] Closed.\n";
    return 0;
}
