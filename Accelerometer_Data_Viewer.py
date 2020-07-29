#Base Python Packages
import threading
import binascii
import logging
import sys
import time
import math
import csv
import os
import glob
#External Python Packages
import pygatt
import pyftdi.serialext
import serial
import serial.tools.list_ports as serialPort
import AnalyzeTest as analyze
import matlab
from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#Constants
sensorsData = "00000000-0000-181C-0001-000000000000"
led = "00000000-0000-181C-0004-000000000000"
command = "00000000-0000-181C-0005-000000000000"
recordCMD = bytearray([0x01])
requestPacketCMD = bytearray([0x08])
stopRecordCMD = bytearray([0x05])

gain = 0x01
agc = 0x00
dataGain = 0
packetNumber = 1
transferComplete = False
packetSign = 0
readpacketSize = 0
crc = 0
timeStamp = 0
recordSize = 0
recordedSize = 0
status = 0
recordID = 0
tagID = 0
bmiEvent = 0
batteryVoltage = 0
ecgSign = 0
ecgSize = 0
ECGData = list(range(300))
AccXData = list(range(25))
AccYData = list(range(25))
AccZData = list(range(25))
GyroXData = list(range(25))
GyroYData = list(range(25))
GyroZData = list(range(25))
factoryECGData = []
factoryDataGain = []
factoryAccXData = []
factoryAccYData = []
factoryAccZData = []
factoryGyroXData = []
factoryGyroYData = []
factoryGyroZData = []
t1 = 0
t2 = 0
packetSize = 936
totalPackets = 150
transferTime = 0
tgcCounter = 0
bluetoothPacket = bytearray()
BlueGigaPort = ''
ftdiCOMPort = ''
threadLock = threading.Lock()
testComplete = False

def analysis(address):
    # Calls matlab engine to analyze data
    analysis = analyze.initialize()
    Test = analysis.AnalyzeTest(str(address))
    analysis.terminate()
    FinalResult = Test[0][0]
    Pass = Test[0][1:11]
    Results = Test[0][11:21]
    return FinalResult, Pass, Results

def data_handler_cb(handle, value):
    # Callback function from subscribe, every notification triggers this
    global packetNumber, packetSize, bluetoothPacket
    if packetNumber == 1:
        packetSize = int.from_bytes(value[2:4], byteorder='little',signed=False)
    bluetoothPacket +=value
    packetNumber += 1
    if packetNumber == math.ceil(packetSize/20)+1:
        packetNumber = 1
        global t2, transferTime, transferComplete
        t2 = time.time()
        transferTime = t2 -t1
        transferComplete = True

def saveData(address,timeOfTest,Data):
    try:
        csvData = 'Data/HELPWEAR_DATA_'+address+'_'+timeOfTest+".csv"
        with open(csvData, 'w') as f:
            writer = csv.writer(f,delimiter='\n')
            writer.writerow(Data)
    except IOError:
        print("I/O error will attempt to try again")
        saveData(address,timeOfTest,Data)
    return csvData
def saveBMIData(address,timeOfTest,factoryAccXData, factoryAccYData, factoryAccZData, factoryGyroXData, factoryGyroYData, factoryGyroZData):
    csvData = 'Data/HELPWEAR_BMI160_DATA_'+address+'_'+timeOfTest+".csv"
    with open(csvData, 'w') as f:
        writer = csv.writer(f,delimiter='\n')
        writer.writerow(factoryAccXData,)
def saveResults(address,timeOfTest,Pass,Results):
    csv_columns = ['GainLevel2','GainLevel3','AGCTest','Impedance','DynamicRange',
                   'FrequencyResponse0_67', 'FrequencyResponse40',
                   'FrequencyResponse60', 'SignalNoise', 'SNR']

    passDict = {'GainLevel2':bool(Pass[0]),'GainLevel3':bool(Pass[1]),
                'AGCTest':bool(Pass[2]),'Impedance':bool(Pass[3]),
                'DynamicRange':bool(Pass[4]),'FrequencyResponse0_67':bool(Pass[5]),
                'FrequencyResponse40':bool(Pass[6]), 'FrequencyResponse60':bool(Pass[7]),
                'SignalNoise':bool(Pass[8]), 'SNR':bool(Pass[9])}
    resultsDict = {'GainLevel2':Results[0],'GainLevel3':Results[1],'AGCTest':Results[2],
                   'Impedance':Results[3],'DynamicRange':Results[4],
                   'FrequencyResponse0_67':Results[5], 'FrequencyResponse40':Results[6],
                   'FrequencyResponse60':Results[7], 'SignalNoise':Results[8], 'SNR':Results[9]}
    try:
        csvResults = 'Results/HELPWEAR_RESULTS_'+address+'_'+timeOfTest+".csv"
        #since it is easier to use Dict write, we first do this, then transpose the result
        with open('temp.csv', 'w') as f:
            d = csv.DictWriter(f,fieldnames=csv_columns)
            d.writeheader()
            d.writerow(passDict)
            d.writerow(resultsDict)
        temp_contents= list(range(3))
        with open('temp.csv','r') as temp:
            temp_contents[0] = temp.readline()
            empty = temp.readline()
            temp_contents[1] = temp.readline()
            empty = temp.readline()
            temp_contents[2] = temp.readline()
        temp_one = temp_contents[0].strip('\n').split(',')
        temp_two = temp_contents[1].strip('\n').split(',')
        temp_three = temp_contents[2].strip('\n').split(',')
        temp_array = [temp_one,temp_two,temp_three]
        out_data = zip(*temp_array)
        with open(csvResults,'w') as final:
            writer = csv.writer(final)
            for row in out_data:
                writer.writerow(row)
        #delete temp to save space
        os.unlink('temp.csv')
    except IOError:
        print("I/O error, will attempt to try again")
        saveResults(address,timeOfTest,Pass,Results)
    return passDict



def process_datapacket(value):
    # Unpacks bluetooth packet into each piece
    global ECGData, AccXData, AccYData, AccZData, GyroXData, GyroYData, GyroZData, dataGain
    global packetSign, readpacketSize, crc, timeStamp, recordSize, tempRecordedSize
    global recordedSize, status, recordID, tagID, bmiEvent, batteryVoltage, ecgSign, ecgSize
    """
    struct BleMap
{
	uint16_t	PacketSign = PACKET_SIGN;		//2
	uint16_t	PacketSize;						//2
	uint16_t 	Crc;							//2
	uint64_t	TimeStamp;						//8
	uint32_t	RecordSize;						//4
	uint32_t	RecordedSize;					//4
	uint16_t	Status;							//2 // not 0 if preview
	union
	{
		uint16_t	BatteryLevel;				// Changed only before sending packet
		uint16_t	RecordId;					// Used internally for mark an unique record
	};
	uint16_t	TagId;							//2
	uint16_t	BMIEvent;						//2
	uint16_t	BatteryVoltage;					//2
	uint16_t	EcgSign = BUFF_SIGN_ECG;		//2
	uint16_t	EcgSize = BUFF_SIZE_ECG_MAX;	//2
	union
	{
		uint16_t	EcgData[BUFF_SIZE_ECG_MAX];				//600
	};

} __attribute__((packed)); // 632 bytes
    """
    #print(packet_number)
    #print(value)
    packetSign = int.from_bytes(value[:2], byteorder='little',signed=False)
    if packetSign != 0x7777:
        signWrong = True
    readpacketSize = int.from_bytes(value[2:4], byteorder='little',signed=False)
    crc = int.from_bytes(value[4:6], byteorder='little',signed=False)
    timeStamp = int.from_bytes(value[6:14], byteorder='little',signed=False)
    recordSize = int.from_bytes(value[14:18], byteorder='little',signed=False)
    recordedSize = int.from_bytes(value[18:22], byteorder='little',signed=False)
    status = int.from_bytes(value[22:24], byteorder='little',signed=False)
    RecordPacket = (status & 0x01) == 0
    dataGain = (status >> 5) & 0x1ff
    batteryChargeLevel = int.from_bytes(value[24:26], byteorder='little',signed=False)
    isCharging = (batteryChargeLevel & 0x80) > 0
    batteryChargeLevel &= 0x7f
    tagID = int.from_bytes(value[26:28], byteorder='little',signed=False)
    bmiEvent = int.from_bytes(value[28:30], byteorder='little',signed=False)
    batteryVoltage = int.from_bytes(value[30:32], byteorder='little',signed=False)
    ecgSign = int.from_bytes(value[32:34], byteorder='little',signed=False)
    if ecgSign == 0xAAAA:
        pass
    ecgSize = int.from_bytes(value[34:36], byteorder='little',signed=False)/2
            #ECGData = list(range(ecgSize))

    ECGData[0] = int.from_bytes(value[36:38], byteorder='little',signed=False)
    ECGData[1] = int.from_bytes(value[38:40], byteorder='little',signed=False)
    j = 40
    for i in range(2,300):
        ECGData[i] = int.from_bytes(value[j:j+2], byteorder='little',signed=False)
        j += 2
    for k in range(0,25):
        AccXData[k] = int.from_bytes(value[j:j+2], byteorder='little',signed=False)
        AccYData[k] = int.from_bytes(value[j+2:j+4], byteorder='little',signed=False)
        AccZData[k] = int.from_bytes(value[j+4:j+6], byteorder='little',signed=False)
        GyroXData[k]  = int.from_bytes(value[j+6:j+8], byteorder='little',signed=False)
        GyroYData[k]  = int.from_bytes(value[j+8:j+10], byteorder='little',signed=False)
        GyroZData[k]  = int.from_bytes(value[j+10:j+12], byteorder='little',signed=False)
        j += 12
    return ECGData, dataGain, AccXData, AccYData, AccZData, GyroXData, GyroYData, GyroZData

# 2 different functions used since first delay is 2
def threadedGainChange2():
    global gain, device, led, threadLock
    gain = 0x02
    threadLock.acquire()
    try:
        device.char_write(led, bytearray([0x00,gain,0x00]), wait_for_response=True)
    except pygatt.exceptions.NotConnectedError:
        print('Device disconnected, Please re-test')
        testComplete = True
    threadLock.release()
    timer2 = threading.Timer(8.0,threadedGainChangeRest)
    timer2.start()

def threadedGainChangeRest():
    global tgcCounter, gain, agc, device, led, threadLock, testComplete
    if tgcCounter == 0:
        gain = 0x03
    if tgcCounter == 1:
        gain = 0x04
    if tgcCounter == 2:
        gain = 0x05
    if tgcCounter == 3:
        agc = 0x01
    if tgcCounter == 4:
        gain = 0x02
        agc = 0x00
    threadLock.acquire()
    try:
        device.char_write(led, bytearray([0x00,gain,agc]), wait_for_response=True)
    except pygatt.exceptions.NotConnectedError:
        print('Device disconnected, Please re-test')
        testComplete = True
    threadLock.release()
    if tgcCounter < 5:
        timer2 = threading.Timer(8.0,threadedGainChangeRest)
        timer2.start()
    tgcCounter += 1


def performanceTest():
    i = 0
    offset = 0
    t = 0
    plt.ion()
    fig=plt.figure()
    x_axis_start = 0
    x_axis_end = 2
    #TEST START
    global testComplete
    timeOfTest = time.strftime("%d%b%Y_%H%M", time.localtime())
    print('Test Start \nProgress:')
    requestPacketNumber = 0
    while(requestPacketNumber < totalPackets):
        # Print progress
        if ((((requestPacketNumber+1)/totalPackets)*100)%10 == 0):
            print(str(((requestPacketNumber+1)/totalPackets)*100)+'%')

        packet = requestPacketNumber.to_bytes(4,byteorder='big')
        timestamp = int(time.time()).to_bytes(8,byteorder='big')

        # Request next packet
        global threadLock
        threadLock.acquire()
        global device
        try:
            device.char_write_long(command, requestPacketCMD+timestamp+packet, wait_for_response=True)
        except pygatt.exceptions.NotConnectedError:
            print('Device disconnected, Please re-test')
            testComplete = True
            break
        threadLock.release()

        # Wait until transfer is complete since we send in 20 Byte chunks
        global t1,transferComplete
        t1 = time.time()
        while(transferComplete == False) or time.time()-t1 < 1:
            time.sleep(0.1)
        transferComplete = False

        # Unpack the data packets
        global bluetoothPacket,factoryECGData,factoryDataGain,factoryAccXData,factoryAccYData,factoryAccZData,factoryGyroXData,factoryGyroYData,factoryGyroZData
        global ECGData, dataGain, AccXData, AccYData, AccZData, GyroXData, GyroYData, GyroZData
        ECGData, dataGain, AccXData, AccYData, AccZData, GyroXData, GyroYData, GyroZData = process_datapacket(bluetoothPacket)

        # Reset bluetoothPacket
        bluetoothPacket.clear()

        # Add data to test data
        factoryECGData.extend(ECGData)

        # Currently we don't do anything with this data
        factoryDataGain.append(dataGain)
        factoryAccXData.extend(AccXData)
        factoryAccYData.extend(AccYData)
        factoryAccZData.extend(AccZData)
        factoryGyroXData.extend(GyroXData)
        factoryGyroYData.extend(GyroYData)
        factoryGyroZData.extend(GyroZData)
        if requestPacketNumber == 0:
            axOffset = AccXData[0]
            ayOffset = AccYData[0]
            azOffset = AccZData[0]
            gxOffset = GyroXData[0]
            gyOffset = GyroYData[0]
            gzOffset = GyroZData[0]
        plt.subplot(311)
        plt.title('ECG')
        if t%600 == 0 and t>1:
            x_axis_start += 2
            x_axis_end += 2
            plt.axis([x_axis_start, x_axis_end, 0, 3.1])
        for data in ECGData:
            plt.scatter(t/300,data/65535*3.1,c='blue')
            t+=1
        plt.subplot(312)
        plt.title('Accelerometer')
        plt.axis([0,150,-1000,1000])
        i = offset
        # /16384 to convert to Gs *9.81 to convert to m/s^2
        for data in AccXData:
            plt.scatter(i/25,(data-32768)/16384,c='red')
            i+=1
        i = offset
        for data in AccYData:
            plt.scatter(i/25,(data-32768)/16384,c='green')
            i+=1
        i = offset
        for data in AccZData:
            plt.scatter(i/25,(data-32768)/16384,c='blue')
            i+=1
        plt.subplot(313)
        plt.title('GyroScope')
        plt.axis([0,150,-1000,1000])
        plt.xlabel('Time')
        plt.ylabel('Data')
        # divide by 16.4 to convert to degrees/s
        for data in GyroXData:
            plt.scatter(i/25,(data-32768)/16.4,c='maroon')
            i+=1
        for data in GyroYData:
            plt.scatter(i/25,(data-32768)/16.4,c='orange')
            i+=1
        for data in GyroZData:
            plt.scatter(i/25,(data-32768)/16.4,c='purple')
            i+=1
        offset += 25
        plt.show()
        plt.pause(0.0001)

        #Integrate Gyro for degrees rotated

        requestPacketNumber += 1
    global address
    #Save data to csv file for matlab to open
    filePath = saveData(address,timeOfTest,factoryECGData)
    saveBMIData(address,timeOfTest,factoryAccXData, factoryAccYData, factoryAccZData, factoryGyroXData, factoryGyroYData, factoryGyroZData)
    #Matlab Python Package to analyze data
    FinalResults, Pass, Results = analysis(filePath)

    #Save Results to CSV
    TestResults = saveResults(address,timeOfTest,Pass,Results)

    #Print Results for user
    print('RESULTS:')
    for key, value in TestResults.items():
        if value == True:
            print("{left_aligned:<25}{right_aligned:>10}".format(left_aligned=key,right_aligned='Passed'))
        else:
            print("{left_aligned:<25}{right_aligned:>10}".format(left_aligned=key,right_aligned='Failed'))
    if FinalResults == 1:
        print('Performance Test Passed')
    else:
        print('Performance Test Failed')
    print('Results Saved Successfully')
    testComplete = True

def main():
    # Enable Logging for debugging purposes
    #logging.basicConfig()
    #logging.getLogger('pygatt').setLevel(logging.DEBUG)
    bgFound = False
    global BlueGigaPort,ftdiCOMPort
    # Detect serial Ports
    while bgFound == False:
        listportinfo = serialPort.comports()
        for port in listportinfo:
            if 'Bluegiga' in str(port):
                BlueGigaPort = str(port.device)
                bgFound = True
            if 'USB Serial Port' in str(port):
                ftdiCOMPort = str(port.device)
                ftdiFound = True
        if not bgFound:
            input('Cannot detect BLE112, Unplug and replug then click enter')

    # Create instance of the bluetooth backend
    bgConnected = False
    # Ensure we can connect to the BLE112
    while(bgConnected==False):
        global adapter
        try:
            # Start the adapter
            adapter = pygatt.BGAPIBackend(serial_port=BlueGigaPort)
            adapter.start()
            bgConnected = True
        except pygatt.backends.bgapi.exceptions.BGAPIError:
            input('Cannot connect to BLE112, Unplug and replug then click enter')
            listportinfo = serialPort.comports()
            for port in listportinfo:
                if 'Bluegiga' in str(port):
                    BlueGigaPort = str(port.device)

    # Test Loop
    continueTesting = ""
    while(continueTesting != "n"):
        global address, device, gain,totalPackets
        connected = False
        while(connected == False):
            address = input("Input MAC address: ")
            if address == 'exit':
                break
            # Get rid of the colons and whitespace if the user inputted it
            address = address.strip().replace(':','')
            try:
                device = adapter.connect(address)
                connected = True
            except pygatt.exceptions.NotConnectedError:
                print('Cannot Connect to Device.  Please try again')

        if connected:
            try:
                # LED and gain settings
                # Byte1 LED, Byte 2 gain, Byte 3 AGC
                gain = 0x01
                device.char_write(led, bytearray([0x02,gain,0x00]), wait_for_response=True)

                # Command bluetooth characterstic
                # <!-- [1b command][8b timestamp][4b record size][6b mac]
                timestamp = int(time.time()).to_bytes(8,byteorder='big')
                device.char_write_long(command, recordCMD+timestamp+bytearray([0xFF,0xFF,0xFF,0xFF]), wait_for_response=True)

                # Subscribe to characterstic when received, prcoess data
                device.subscribe(sensorsData, callback=data_handler_cb,indication=False,wait_for_response=False)

                #Start requesting data after 1 second
                testTimer = threading.Timer(1.0,performanceTest)
                testTimer.start()

                #Start Gain Change Timer which activates after 2 seconds
                global tgcCounter
                tgcCounter = 0
                timer = threading.Timer(2.0,threadedGainChange2)
                timer.start()

                #wait for test to finish
                global testComplete
                while(testComplete != True):
                    time.sleep(0.1)
                testComplete = False


            except pygatt.exceptions.NotConnectedError:
                print('Device disconnected, Please re-test')

            finally: #this code runs regardless of crash or not
                #Stop recording
                device.char_write(command, stopRecordCMD, wait_for_response=True)
                device.unsubscribe(sensorsData, wait_for_response=False)

                #Disconnect from device
                device.disconnect()

                #reset data
                global factoryECGData,factoryDataGain,factoryAccXData,factoryAccYData,factoryAccZData,factoryGyroXData,factoryGyroYData,factoryGyroZData
                factoryECGData.clear()
                factoryDataGain.clear()
                factoryAccXData.clear()
                factoryAccYData.clear()
                factoryAccZData.clear()
                factoryGyroXData.clear()
                factoryGyroYData.clear()
                factoryGyroZData.clear()


        continueTesting = input("Press Enter to test another device, n to stop program\n")

    #Exited the main loop, stop the adapter session
    adapter.stop()
    #ftdi.close()
    upload = input('Do you want to upload results? y/n\n')
    if upload == 'y':
        #Login to Google Drive and create drive object
        g_login = GoogleAuth()
        g_login.LoadCredentialsFile("credentials.txt")
        if g_login.credentials is None:
            # Authenticate if they're not there
            g_login.LocalWebserverAuth()
        elif g_login.access_token_expired:
            # Refresh them if expired
            g_login.Refresh()
        else:
            # Initialize the saved creds
            g_login.Authorize()
        # Save the current credentials to a file
        g_login.SaveCredentialsFile("credentials.txt")
        drive = GoogleDrive(g_login)

        # Importing os and glob to find all PDFs inside subfolder
        team_drive_id = '1zoEZV3kp2_MfGshZCeYLCX8cXzgYD0mL'
        Data_folder_id = '1maJ30HsK5QccW0AU7QvMZNRc5TUpHdDX'
        Results_folder_id = '1eWku_AnBpsic2RW5Ojqw3p_nuPkcNOPf'
        owd = os.getcwd()
        os.chdir("Data")
        print('Uploading Data files')
        for file in glob.glob("*.csv"):
            with open(file,"r") as f:
                fn = os.path.basename(f.name)
                file_drive = drive.CreateFile({'title': fn,
                'parents': [{
                    'kind': 'drive#fileLink',
                    'teamDriveId': team_drive_id,
                    'id': Data_folder_id}]})
                file_drive.SetContentString(f.read())
                file_drive.Upload(param={'supportsTeamDrives': True})

        os.chdir(owd)
        os.chdir("Results")
        print('Uploading Result files')
        for file in glob.glob("*.csv"):
            with open(file,"r") as f:
                fn = os.path.basename(f.name)
                file_drive = drive.CreateFile({'title': fn,
                'parents': [{
                    'kind': 'drive#fileLink',
                    'teamDriveId': team_drive_id,
                    'id': Results_folder_id}]})
                file_drive.SetContentString(f.read())
                file_drive.Upload(param={'supportsTeamDrives': True})

        # Delete Files?
        #delete = input('Upload Successful, Delete ALL Data and Result files? y/n\n')
        #if delete == 'y':
        #    filelist = glob.glob("*.csv")
        #    for f in filelist:
        #        os.remove(f)
        #    os.chdir(owd)
        #    os.chdir("Data")
        #    filelist = glob.glob("*.csv")
        #    for f in filelist:
        #        os.remove(f)
if __name__ == '__main__':
    exit(main())
