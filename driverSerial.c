#include <stdio.h>
#include <string.h> // memset()

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close(), sleep()
#include <sys/select.h>

#include <time.h> // Used to time functions

#define PORT "/dev/ttyACM1"

int serial_port; // file descriptor del PORT
char buf[128];

void writeSerial(unsigned char msg)
{
    // writes just one byte to serial, not optimal, just to "fair" comparision with the Python program
    unsigned char buf[1] = {msg};
    write(serial_port, buf, sizeof(msg));
}

// returns the number of written bytes
int readSerial()
{
    // set all buffer to \0
    memset(&buf, '\0', sizeof(buf));
    // n number of bytes read
    int n = read(serial_port, &buf, sizeof(buf));
    if (n == 0)
    {
        printf("No bytes recived\n");
        return 0;
    }
    else
    {
        // haddle input data
        printf("Read %i bytes. First received message: %x\n", n, (unsigned char)buf[0]);
    }
    return n;
}

void orderAndAck()
{
    // Orden 0x00 - ACK debe ser 0xAA
    writeSerial(0x00);
    fd_set rfds;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(serial_port, &rfds);

    // Asignamos 2 segundos de timeout para la función select()
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    retval = select(serial_port + 1, &rfds, NULL, NULL, &tv);
    if (retval < 0)
    {
        puts("Error on select in dataRequest()");
    }
    else
    {
        // TODO: gestionar si recibimos más de un dato por error
        readSerial();
        unsigned char resp = buf[0];
        // si es diferente de 0xAA entonces hemos leido mal
        printf("Leido: 0x%02X\n", (unsigned char)resp);
        if (resp != (unsigned char)0xAA)
        {
            printf("Error, leido: 0x%02x\n", resp);
        }
    }
}

unsigned char *dataRequest()
{
    // https://man7.org/linux/man-pages/man2/select.2.html
    fd_set rfds; // needed for select
    writeSerial(0x01); // we want 2 bytes answer
    static unsigned char responses[2];
    int bufIdx = 0;
    int responsesIdx = 0;

    // TODO: Add while loop error e.g. not getting final msg
    // when we have final message we will return
    while (1)
    {
        int bytesRead;

        FD_ZERO(&rfds);
        FD_SET(serial_port, &rfds); // archivo que queremos monitorizar

        // Asignamos 2 segundos de timeout para la función select()
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int retval = select(serial_port + 1, &rfds, NULL, NULL, &tv);

        if (retval < 0)
        {
            puts("Error on select in dataRequest()");
        }
        else
        {
            bytesRead = readSerial(); // el valor retornado debe ser > 0
            while (bytesRead != 0)    // Used to read all inputs values
            {
                unsigned char resp = buf[bufIdx++];
                // end data
                if (resp == 0xFF)
                {
                    return responses;
                }
                else
                {
                    responses[responsesIdx++] = resp;
                }
                // counter of remaining data
                bytesRead -= 1;
            }
        }
    }
}

void nResponses(unsigned char msg)
{
    char nResponses[(int)msg];
    int idx = 0;
    writeSerial(msg);

    fd_set rfds;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(serial_port, &rfds);

    // Asignamos 2 segundos de timeout para la función select()
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    // We will receive the number of msgs told in the sent msg
    while (idx != msg+1)
    {
        // wait for some change in serial
        retval = select(serial_port + 1, &rfds, NULL, NULL, &tv);
        if (retval < 0)
        {
            puts("Error on select in dataRequest()");
        } else {
            readSerial();
            while(idx < msg+1) {
                
                if ((unsigned char) buf[idx] == 0xFF){
                    return;
                }
                printf("msg: %x idx %d - 0x%02x \n", msg, idx, buf[idx]);
                
                // if the msg is 0x11 we mark the idx with it
                if (buf[idx] == (unsigned char) 0x11)
                {
                    // TODO process nResponses array to check if everything was okay instead of printing all msgs
                    nResponses[idx] = buf[idx];
                }
                else
                {
                    printf("Fail %c respuesta %x\n", msg, buf[0]);
                }
                idx += 1;
            }
        }
    }
}

void closeAndOpen()
{
    close(serial_port);
    serial_port = open(PORT, O_RDWR);
}

double timeAvg(double times[], int nElements)
{
    double sum = 0;
    for (int i = 0; i < nElements; i++)
    {
        sum += times[i];
    }
    return sum / nElements;
}

int main()
{
    // opening tty file and configuring it from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    serial_port = open(PORT, O_RDWR); // port file descriptor
    if (serial_port < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    // Arduino resets itself after opening file so we wait it
    sleep(2);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Read settings and edit them
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    int nElements;

    int orderAndAckNumExecutions = 5;
    double orderAndAckTimes[orderAndAckNumExecutions];
    for (int i = 0; i < orderAndAckNumExecutions; i++) {
        clock_t startTime = clock();
        orderAndAck();
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * 1000.0 / CLOCKS_PER_SEC;
        orderAndAckTimes[i] = elapsed;
    }
    nElements = sizeof(orderAndAckTimes)/sizeof(double);
    printf("Average time for orderAndAck: %f ms\n", timeAvg(orderAndAckTimes, nElements));

    puts("\n--------------------------\n");

    unsigned char *p;
    
    int dataRequestExecutions = 10;
    double dataRequestTimes[dataRequestExecutions];
    for (int i = 0; i < dataRequestExecutions; i++) {
        clock_t startTime = clock();
        p = dataRequest();
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * 1000.0 / CLOCKS_PER_SEC;
        dataRequestTimes[i] = elapsed;
        printf("Leido dataRequest: 0x%02X 0x%02X\n", p[0], p[1]);
    }
    nElements = sizeof(dataRequestTimes)/sizeof(double);
    printf("Average time for orderAndAck: %f ms\n", timeAvg(dataRequestTimes, nElements));

    puts("\n--------------------------\n");
    
    int nResponsesExecutions = 1;
    double nResponsesTimes[nResponsesExecutions];
    for (int i = 0; i < nResponsesExecutions; i++) {
        clock_t startTime = clock();
        nResponses(10);
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * 1000.0 / CLOCKS_PER_SEC;
        nResponsesTimes[i] = elapsed;
    }
    nElements = sizeof(nResponsesTimes)/sizeof(double);
    printf("Average time for orderAndAck: %f ms\n", timeAvg(nResponsesTimes, nElements));
}