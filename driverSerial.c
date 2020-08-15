#include <stdio.h>
#include <string.h> // memset()

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close(), sleep()
#include <sys/select.h>

#include <time.h> // Used to time functions

#define PORT "/dev/ttyACM0"
#define N_EXECUTIONS 100

int serial_port; // file descriptor del PORT
char buf[128];

void writeSerial(unsigned char msg);
int readSerial();
void orderAndAck();
unsigned char *dataRequest();
void nResponses(unsigned char msg);
double nResponsesBenchmark(int nResponsesExecutions, int numberOfResponses);
void closeAndOpen();
double timeAvg(double times[], int nElements);

void writeSerial(unsigned char msg)
{
    // writes just one byte to serial, not optimal, just to "fair" comparision with the Python program
    unsigned char buf[1] = {msg};
    write(serial_port, buf, sizeof(msg));
}

// returns the number of written bytes
int readSerial()
{
    /*
    Reads PORT(defined above) file and save the results in buf array
    returns: number of bytes that has been read from serial PORT.
    */
    // set all buffer to \0
    memset(&buf, '\0', sizeof(buf));
    // n number of bytes read
    int n = read(serial_port, &buf, sizeof(buf));
    if (n == 0)
    {
        printf("No bytes recived\n");
        return 0;
    }
    return n;
}

void orderAndAck()
{
    /*
    Function that sends 0x00 and check if it receives from microcontroller 0xAA
    */
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
        int nReadBytes = readSerial();
        if (nReadBytes == 1) {
            unsigned char resp = buf[0];
            // si es diferente de 0xAA entonces hemos leido mal
            printf("Leido: 0x%02X\n", (unsigned char)resp);
            if (resp != (unsigned char)0xAA)
            {
                printf("Error, leido: 0x%02x\n", resp);
            }
        } else if (nReadBytes == 0) {
            puts("No received data");
        } else if (nReadBytes >= 2) {
            puts("Received more than one byte");
        } else {
            puts("Leido un dato más del solicitado");
        }
    }
}

unsigned char *dataRequest()
{
    /*
    Function to trigger 0x01 in microcontroller, it will ask for data
    and micro will answer with 0xF0, 0xF2 and 0xFF to end
    */
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
                    // return pointer to array
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
    /*
    function to trigger n responses from microcontroller
    args:
    msg: must be bigger than 0x02 to trigger the microcontroller function 
    */
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

    int totalRead = 0;
    // We will receive the number of msgs told in the sent msg
    // wait for some change in serial
    retval = select(serial_port + 1, &rfds, NULL, NULL, &tv);
    if (retval < 0)
    {
        puts("Error on select in nResponses()");
    } else {
        int nBytes = readSerial(); // number of read bytes so we can know when to stop reading the buffer
        // last idx we read serial
        int lastRead = 0;

        while(totalRead < msg+1) {
            // We need to keep track if the idx is the same of number of read bytes
            // we use totalRead as idx in the data storage array
            if (idx == nBytes) {
                idx = 0;
                nBytes = readSerial();
            }

            unsigned char resp = buf[idx]; // response
            if (resp == (unsigned char) 0x11)
            {
                nResponses[totalRead] = buf[idx];
            }
            else if (resp == (unsigned char) 0xff)
            {
                printf("nResponses(%u) finalizado exitosamente\n", msg);
            }
            else
            {
                printf("Fail %u respuesta 0x%02x\n", msg, resp);
            }
            idx += 1;
            totalRead += 1;
        }
    }
}

double nResponsesBenchmark(int nResponsesExecutions, int numberOfResponses){
    /*
    Function to test nResponses() time
    args:
    nResponsesExecutions: number of times we want to execute the function and then do the average
    numberOfResponses: number of consecutives responses we want from the microcontroller
    returns: average time taken in ms
    */
    double nResponsesTimes[nResponsesExecutions];
    for (int i = 0; i < nResponsesExecutions; i++) {
        clock_t startTime = clock();
        nResponses(numberOfResponses);
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * (1000.0 / CLOCKS_PER_SEC);
        nResponsesTimes[i] = elapsed;
    }
    float avgTime = timeAvg(nResponsesTimes, nResponsesExecutions);
    printf("Average time for nResponses(%u): %f ms\n", numberOfResponses, avgTime);
    return avgTime;
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

    FILE *dataFile;
    dataFile = fopen("data/cData.txt", "w");

    int orderAndAckNumExecutions = N_EXECUTIONS;
    double orderAndAckTimes[orderAndAckNumExecutions];
    for (int i = 0; i < orderAndAckNumExecutions; i++) {
        clock_t startTime = clock();
        orderAndAck();
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * 1000.0 / CLOCKS_PER_SEC;
        orderAndAckTimes[i] = elapsed;
    }
    float orderAndAckTime = timeAvg(orderAndAckTimes, orderAndAckNumExecutions);
    printf("Average time for orderAndAck: %f ms\n", orderAndAckTime);
    fprintf(dataFile, "%.2f\n", orderAndAckTime);
    puts("\n--------------------------\n");

    unsigned char *p;
    
    int dataRequestExecutions = N_EXECUTIONS;
    double dataRequestTimes[dataRequestExecutions];
    for (int i = 0; i < dataRequestExecutions; i++) {
        clock_t startTime = clock();
        p = dataRequest();
        clock_t endTime = clock();
        double elapsed = (double)(endTime-startTime) * (1000.0 / CLOCKS_PER_SEC);
        dataRequestTimes[i] = elapsed;
        printf("Leido dataRequest: 0x%02X 0x%02X\n", p[0], p[1]);
    }
    float avgTimeDataRequest = timeAvg(dataRequestTimes, dataRequestExecutions);
    printf("Average time for dataRequest(): %f ms\n", avgTimeDataRequest);
    fprintf(dataFile, "%.2f\n", avgTimeDataRequest);
    puts("\n--------------------------\n");
    
    fprintf(dataFile, "%.2f\n", nResponsesBenchmark(N_EXECUTIONS, 20));
    fprintf(dataFile, "%.2f\n", nResponsesBenchmark(N_EXECUTIONS, 60));
    fprintf(dataFile, "%.2f\n", nResponsesBenchmark(N_EXECUTIONS, 100));
    fprintf(dataFile, "%.2f\n", nResponsesBenchmark(N_EXECUTIONS, 200));
}