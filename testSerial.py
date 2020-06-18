import serial
import time
import timeit

PORT = '/dev/ttyACM1'
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE)

def write (input):
    ser.write(input.decode("hex"))

def orderAndAck():
    ser.write(b'\x00')
    # https://stackoverflow.com/questions/17553543/pyserial-non-blocking-read-loop/38758773#38758773
    while ser.in_waiting == 0:
        # if(ser.in_waiting != 0):
        #     print(ser.read(ser.in_waiting))
        pass
    rec = ser.read()
    # print(ser.in_waiting)
    if (rec != b'\xAA'):
        print(f"Fail {rec}")

def dataRequest():
    ser.reset_input_buffer()
    # "hardcoded" 2 valores de respuesta y fin(0xFF)
    ser.write(b'\x01')
    responses = []
    while True:
        while ser.in_waiting == 0:
            pass
        resp = ser.read()
        if resp == b'\xFF':
            return responses
        else:
            responses.append(resp)
            print(responses)
        

def reqAndResp(numberOfRequest):
    for _ in range(numberOfRequest):
        ser.write(b'\x00')
        while ser.in_waiting == 0:
            pass
        rec = ser.read()
        # print(rec)
        if (rec != b'\xAA'):
            print("Fail")


def nResponses(numberOfResponses):
    responses = []
    ser.write(numberOfResponses.to_bytes(1, "big"))
    while len(responses) != numberOfResponses:
        while ser.in_waiting == 0:
            pass
        resp = ser.read()
        if (resp == b'\x11'):
            responses.append(resp)
        else:
            print(f"Fail {numberOfResponses} resp {resp}")
        # TODO: Add timeout

def closeAndOpen():
    global ser
    ser.close()
    ser = serial.Serial(PORT, BAUDRATE)

if __name__ == "__main__":
    # intercambio rapido una sola orden y ack
    print("Time orderAndAck():")
    print(timeit.timeit("orderAndAck()", setup="from testSerial import orderAndAck, ser", number=5))
 
    time.sleep(1)
    # request de datos
    # print(dataRequest())
    # print(timeit.timeit("dataRequest()", setup="from __main__ import dataRequest"))

    # 20 respuestas 0x14
    print("Read 20 responses\n")
    print(timeit.timeit("nResponses(20)", setup="from __main__ import nResponses", number=1))

    # 60 respuestas 0x3C
    print("Read 60 responses\n")
    print(timeit.timeit("nResponses(60)", setup="from __main__ import nResponses", number=1))

    # 100 respuestas 0x64
    print("Read 100 responses\n")
    print(timeit.timeit("nResponses(100)", setup="from __main__ import nResponses", number=1))
    
    # 200 respuestas 0xC8
    print("Read 200 responses\n")
    print(timeit.timeit("nResponses(200)", setup="from __main__ import nResponses", number=1))

    # abrir y cerrar puerto
    closeAndOpen()
