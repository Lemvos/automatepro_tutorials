#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

bool running = true;

void signal_handler(int) {
    running = false;
    std::cout << "\n[Receiver] Stopping...\n";
}

// Configure serial port settings
bool configure_serial_port(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("[Receiver] tcgetattr");
        return false;
    }

    cfsetospeed(&tty, B9600); // Baudrate
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[Receiver] tcsetattr");
        return false;
    }

    return true;
}

int main() {
    signal(SIGINT, signal_handler);

    const char* device = "/dev/ttyUSB0";
    std::cout << "[Receiver] Opening serial port: " << device << std::endl;

    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("[Receiver] open");
        return 1;
    }

    if (!configure_serial_port(fd)) {
        close(fd);
        return 1;
    }

    std::cout << "[Receiver] Serial port ready. Press Ctrl+C to stop.\n";

    uint8_t buffer[2];
    while (running) {
        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            std::cout << "[Receiver] Received: ";
            for (ssize_t i = 0; i < bytes_read; ++i) {
                std::cout << static_cast<int>(buffer[i]) << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "[Receiver] No data received.\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(fd);
    std::cout << "[Receiver] Closed.\n";
    return 0;
}
