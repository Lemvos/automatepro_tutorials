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
    std::cout << "\n[Sender] Stopping...\n";
}

// Set up serial port attributes
bool configure_serial_port(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("[Sender] tcgetattr");
        return false;
    }

    cfsetospeed(&tty, B9600); // Set baudrate
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 10;           // 1s timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD); // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // no flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[Sender] tcsetattr");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    const char* device = "/dev/ttyUSB0";

    std::cout << "[Sender] Opening serial port: " << device << std::endl;

    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("[Sender] open");
        return 1;
    }

    if (!configure_serial_port(fd)) {
        close(fd);
        return 1;
    }

    std::cout << "[Sender] Serial port ready. Press Ctrl+C to stop.\n";

    int counter = 0;
    while (running) {
        uint8_t data[2];
        data[0] = counter & 0xFF;
        data[1] = (counter >> 8) & 0xFF;

        ssize_t bytes_written = write(fd, data, 2);
        if (bytes_written == 2) {
            std::cout << "[Sender] Sent Data: " << static_cast<int>(data[0]) << " " << static_cast<int>(data[1]) << std::endl;
        } else {
            perror("[Sender] Write failed");
        }

        counter++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(fd);
    std::cout << "[Sender] Closed.\n";
    return 0;
}
