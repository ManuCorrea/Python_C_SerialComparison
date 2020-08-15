import serial
import time
import timeit

PORT = '/dev/ttyACM0'
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE)

def write (input):
    ser.write(input.decode("hex"))

def orderAndAck():
    """
    Function that sends 0x00 and check if it receives from microcontroller 0xAA
    """
    ser.write(b'\x00')
    rec = ser.read()
    # print(ser.in_waiting)
    if (rec != b'\xAA'):
        print(f"Fail {rec}")

def dataRequest():
    """
    Function to trigger 0x01 in microcontroller, it will ask for data
    and micro will answer with 0xF0, 0xF2 and 0xFF to end
    """
    ser.reset_input_buffer()
    # "hardcoded" 2 valores de respuesta y fin(0xFF)
    ser.write(b'\x01')
    responses = []
    while True:
        resp = ser.read()
        if resp == b'\xFF':
            return responses
        else:
            responses.append(resp)        

def nResponses(numberOfResponses):
    """
    function to trigger n responses from microcontroller
    args:
    numberOfResponses: must be bigger than 0x02 to trigger the microcontroller function 
    """
    responses = []
    ser.write(numberOfResponses.to_bytes(1, "big"))
    cont = 0
    while len(responses) != numberOfResponses:
        while ser.in_waiting == 0:
            pass
        resp = ser.read()
        cont += 1
        if (resp == b'\x11'):
            responses.append(resp)
        else:
            responses.append(resp)
            print(f"Fail {numberOfResponses} resp {resp} index {cont}")
        # TODO: Add timeout
    if ser.read() == b'\xff':
        print("finalizacion exitosa")

def closeAndOpen():
    global ser
    ser.close()
    ser = serial.Serial(PORT, BAUDRATE)

if __name__ == "__main__":
    timesResults = []
    time.sleep(2)
    N_EXECUTIONS = 100
    # intercambio rapido una sola orden y ack
    orderAndAckTime = (timeit.timeit("orderAndAck()", setup="from testSerial import orderAndAck, ser", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time orderAndAck(): " + str(orderAndAckTime))
    timesResults.append(orderAndAckTime)
 
    time.sleep(1)
    # request de datos
    dataRequestTime = (timeit.timeit("dataRequest()", setup="from __main__ import dataRequest", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time dataRequest(): " + str(dataRequestTime))
    timesResults.append(dataRequestTime)

    # 20 respuestas 0x14
    nResponsesTime = (timeit.timeit("nResponses(20)", setup="from __main__ import nResponses", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time nResponses(20): " + str(nResponsesTime))
    timesResults.append(nResponsesTime)

    # 60 respuestas 0x3C
    nResponsesTime = (timeit.timeit("nResponses(60)", setup="from __main__ import nResponses", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time nResponses(60): " + str(nResponsesTime))
    timesResults.append(nResponsesTime)

    # 100 respuestas 0x64
    nResponsesTime = (timeit.timeit("nResponses(100)", setup="from __main__ import nResponses", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time nResponses(100): " + str(nResponsesTime))
    timesResults.append(nResponsesTime)

    # 200 respuestas 0xC8
    nResponsesTime = (timeit.timeit("nResponses(200)", setup="from __main__ import nResponses", number=N_EXECUTIONS) * 1000) / N_EXECUTIONS
    print("Time nResponses(200): " + str(nResponsesTime))
    timesResults.append(nResponsesTime)

    with open("data/pythonData.txt", "w") as file:
        for time in timesResults:
            file.write(str(round(time, 2)) + "\n")

    # abrir y cerrar puerto
    closeAndOpen()
