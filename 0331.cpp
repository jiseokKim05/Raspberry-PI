#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>

class UART {
private:
    int uart_fd;
    pthread_t read_thread;
    unsigned char read_buf[7]; // 버퍼 크기는 최소 7바이트 (#S4321* 길이)
    bool running;
    const char* serial_device;
    speed_t baudrate;

    static uint16_t CRC16_MODBUS(const unsigned char* data, int length) {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; i++) {
            crc ^= (uint16_t)data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }

    static void* readThreadWrapper(void* instance) {
        return ((UART*)instance)->readThreadFunction();
    }

    void* readThreadFunction() {
        int num_bytes = -1;
        unsigned char insert_buf;
        printf("Thread started for %s\n", serial_device);

        while (running) {
            while ((num_bytes = read(uart_fd, &insert_buf, 1)) > 0) {
                printf("[%s] Data read: %d bytes, Value: %02X\n", 
                       serial_device, num_bytes, insert_buf);

                // 버퍼를 왼쪽으로 이동하고 새로운 바이트 추가
                for (int i = 0; i < 6; i++) {
                    read_buf[i] = read_buf[i + 1];
                }
                read_buf[6] = insert_buf;

                // 모터 명령 파싱 시도
                if (parseMotorCommand(read_buf, 7)) {
                    printf("[%s] Motor command detected: ", serial_device);
                    for (int i = 0; i < 7; i++) {
                        printf("%c", read_buf[i]); // ASCII 문자로 출력
                    }
                    printf("\n");
                }
            }
        }
        return NULL;
    }

public:
    UART(const char* device = "/dev/ttyS0", speed_t baud = B9600)
        : uart_fd(-1), running(false), serial_device(device), baudrate(baud) {
        memset(read_buf, 0, sizeof(read_buf));
    }

    ~UART() {
        close();
    }

    bool init() {
        uart_fd = open(serial_device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd < 0) {
            printf("Error opening %s: %s\n", serial_device, strerror(errno));
            return false;
        }

        struct termios tty;
        if (tcgetattr(uart_fd, &tty) != 0) {
            printf("Error %i from tcgetattr for %s: %s\n", 
                   errno, serial_device, strerror(errno));
            close();
            return false;
        }

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 10;    // 1초 타임아웃
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr for %s: %s\n", 
                   errno, serial_device, strerror(errno));
            close();
            return false;
        }
        return true;
    }

    bool startReadThread() {
        if (uart_fd < 0) {
            printf("Serial port %s not initialized\n", serial_device);
            return false;
        }

        running = true;
        if (pthread_create(&read_thread, NULL, readThreadWrapper, this) != 0) {
            printf("Failed to create read thread for %s\n", serial_device);
            running = false;
            return false;
        }
        return true;
    }

    bool parseMotorCommand(const unsigned char* data, int data_length) {
        if (data_length < 7) return false;

        // 패턴 정의: #S4321*와 #S1234*
        const unsigned char pattern1[7] = {'#', 'S', '4', '3', '2', '1', '*'};
        const unsigned char pattern2[7] = {'#', 'S', '1', '2', '3', '4', '*'};

        // 패턴과 일치하는지 확인
        bool match1 = memcmp(data, pattern1, 7) == 0;
        bool match2 = memcmp(data, pattern2, 7) == 0;

        return match1 || match2;
    }

    int write(const unsigned char* data, int length) {
        if (uart_fd < 0) {
            return -1;
        }
        return ::write(uart_fd, data, length);
    }

    void close() {
        if (running) {
            running = false;
            pthread_join(read_thread, NULL);
        }
        if (uart_fd >= 0) {
            ::close(uart_fd);
            uart_fd = -1;
        }
    }

    const unsigned char* getReadBuffer() const {
        return read_buf;
    }
};

// 테스트용 데이터 전송 함수
void simulateData(UART& uart1, UART& uart2) {
    unsigned char cmd1[] = {'#', 'S', '4', '3', '2', '1', '*'}; // #S4321*
    unsigned char cmd2[] = {'#', 'S', '1', '2', '3', '4', '*'}; // #S1234*
    unsigned char noise[] = {'A', 'B', 'C'}; // 잡음 데이터

    sleep(1);
    uart1.write(noise, 3); // 잡음 데이터 전송
    sleep(1);
    uart1.write(cmd1, 7);  // #S4321* 전송
    sleep(1);
    uart2.write(cmd2, 7);  // #S1234* 전송
}

int main() {
    UART uart1("/dev/ttyS0", B115200);
    UART uart2("/dev/ttyUSB0", B115200);

    if (!uart1.init()) {
        printf("Failed to initialize UART for /dev/ttyS0\n");
        return -1;
    }
    if (!uart2.init()) {
        printf("Failed to initialize UART for /dev/ttyUSB0\n");
        return -1;
    }

    if (!uart1.startReadThread()) {
        printf("Failed to start read thread for /dev/ttyS0\n");
        uart1.close();
        return -1;
    }
    if (!uart2.startReadThread()) {
        printf("Failed to start read thread for /dev/ttyUSB0\n");
        uart1.close();
        uart2.close();
        return -1;
    }

    printf("Both UART ports initialized and reading threads started.\n");
    printf("Press Ctrl+C to exit...\n");

    // 시뮬레이션 데이터 전송
    simulateData(uart1, uart2);

    while (1) {
        sleep(1);
    }

    uart1.close();
    uart2.close();
    return 0;
}
