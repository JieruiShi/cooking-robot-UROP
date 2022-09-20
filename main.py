import UR_control
import Daq_control
import Arduino_control
import time
import numpy as np
import keyboard

class cookingRobot(object):

    def __init__(self, enableArduino = True, enableDaq = True, enableUR = True):
        self.dataBuffer = [] #data buffer format: (relative time, distance, saltOn, waterOn, pHRaw, temperatureRaw, conductanceRaw)
        self.dataFilePath = "cookingRobotData--{date}-{no}".format(date = "0910", no = 4)
        self.cookingStartTime = None # To be set using the startCooking function
        self.cooking = False
        self.emergency = False # When emergency is triggered, stop what the arm is doing, and close salt and water
        self.enablePrint = True
        self.enableFeedback = True
        self.counter = 0
        self.counters = {} # Whenever invoking a multi-step action, create a counter in counters, upon finishing, destroy the counter
        self.conductanceTask = {
            "startTime": 0,
            "lastTime": 1,
            "waitTime": 39,
            "state": 2,
            "height": 2
        }
        self.rotateTask = {
            "startTime": 0,
            "waitTime": 35,
            "angleValues": [0, 45, 90, 135, 90, 45, 0, -45, -90, -45],
            "state": 0
        }
        self.feedbackTask = {
            "startTime": 0,
            "waitTime": 60,
            "kpH": 1600.0,
            "kC": 40.0
        }
        self.pHdataBuffer = {
            "pHraw": [],
            "currentpH": 0
        }
        self.conductanceDataBuffer = {
            "conductanceRaw": [],
            "currentConductances": [0],
            "currentTime": 0
        }
        self.pHref = [1.8085, 5.411e-5, -1.358e-8, 1.036e-12]
        self.Cref = []

        if enableArduino:
            self.A = Arduino_control.ArduinoLink()
        if enableDaq:
            self.D = Daq_control.DaqLink()
        if enableUR:
            self.UR = UR_control.URLink()

        time.sleep(3)

        if enableArduino and enableDaq and enableUR:
            print("------initialization successful!!------")

    def saveBufferToFile(self, writeLine = 100):
        """When the buffer reaches 1000 records or cooking terminates, call this function, and writes the first 1000 lines in dataBuffer into output file, in npy format"""
        if len(self.dataBuffer) < writeLine:
            toBuffer = self.dataBuffer
            self.dataBuffer = []
        else:
            toBuffer = self.dataBuffer[:writeLine]
            self.dataBuffer = self.dataBuffer[writeLine:] # removes first 1000/writeLine worth data from the buffer

        appendData = np.array(toBuffer, dtype=float)

        try:
            loadedData = np.load("{}.npy".format(self.dataFilePath))
            appendedData = np.append(loadedData,appendData, axis = 0)
        except:
            appendedData = appendData

        np.save(self.dataFilePath, appendedData)
        print("successfully saved data, file currently holds {} records".format(str(len(appendedData))))

    def startCooking(self):
        self.cookingStartTime = time.time()
        self.cooking = True
        self.A.clearSerialBuffer() #clear buffer before starting to intake data
        header = np.array([self.cookingStartTime],dtype = np.double)
        np.save("{}-header".format(self.dataFilePath), header)

    def resumeCooking(self):
        header = np.load("{}-header.npy".format(self.dataFilePath))
        self.cookingStartTime = header[0]
        print("resuming cooking at {} seconds".format(time.time()-self.cookingStartTime))
        self.cooking = True
        self.A.clearSerialBuffer()
        # self.waitConductanceTask()

    def endCooking(self):
        self.cooking = False
        self.saveBufferToFile(writeLine = 10000) #write the remaining data in the buffer to npy file

    def adjustHeight(self, targetHeight):
        try:
            heightData = [dataRow[1] for dataRow in self.dataBuffer[-10:]]
            currentHeight = sum(heightData)/len(heightData)
            relativeHeightMovement = currentHeight - targetHeight
            if abs(relativeHeightMovement) > 0.1:
                self.UR.adjustToolPosition([0,0,relativeHeightMovement/100])
                print("moving to height: {}cm".format(targetHeight))
        except:
            print("error occured when trying to adjust height, action abandoned!")
            pass

    def startConductanceTask(self):
        self.UR.adjustToolPosition([0,0, self.conductanceTask["height"]/100])
        self.conductanceTask["state"] = 1
        self.conductanceTask["startTime"] = time.time() - self.cookingStartTime

    def waitConductanceTask(self):
        self.UR.adjustToolPosition([0, 0, -1 * self.conductanceTask["height"]/100])
        self.conductanceTask["state"] = 0
        self.conductanceTask["startTime"] = time.time() - self.cookingStartTime

    def doRotateTask(self):
        targetAngle = self.rotateTask["angleValues"][self.rotateTask["state"]]
        self.adjustEndRotation(targetAngle)
        self.rotateTask["state"] = (self.rotateTask["state"] + 1) % 10
        self.rotateTask["startTime"] = time.time() - self.cookingStartTime

    def taskStep(self, relativeTime):
        if self.conductanceTask["state"] == 0:
            if relativeTime - self.conductanceTask["startTime"] > self.conductanceTask["waitTime"]:
                self.startConductanceTask()
        elif self.conductanceTask["state"] == 1:
            if relativeTime - self.conductanceTask["startTime"] > self.conductanceTask["lastTime"]:
                self.waitConductanceTask()

        if relativeTime - self.rotateTask["startTime"] > self.rotateTask["waitTime"]:
            self.doRotateTask()

        if self.enableFeedback:
            if relativeTime - self.feedbackTask["startTime"] > self.feedbackTask["waitTime"]:
                if relativeTime > 300:
                    self.proportionalFeedbackTask()

    def measurementStep(self, pHraw, conductanceRaw, relativeTime):
        if self.conductanceTask["state"] != 1 and pHraw > 1.8 and pHraw < 1.95:
            self.pHdataBuffer["pHraw"].append(pHraw)
            if len(self.pHdataBuffer["pHraw"]) > 500:
                self.pHdataBuffer["pHraw"] = self.pHdataBuffer["pHraw"][1:]
                self.pHdataBuffer["currentpH"] = sum(self.pHdataBuffer["pHraw"])/len(self.pHdataBuffer["pHraw"])

        if self.conductanceTask["state"] == 1:
            if relativeTime - self.conductanceDataBuffer["currentTime"] > 2:
                self.conductanceDataBuffer["currentTime"] = relativeTime
                self.conductanceDataBuffer["conductanceRaw"] = [conductanceRaw]
                self.conductanceDataBuffer["currentConductances"].append(sum(self.conductanceDataBuffer["conductanceRaw"])/len(self.conductanceDataBuffer["conductanceRaw"]))
                if len(self.conductanceDataBuffer["currentConductances"]) > 3:
                    self.conductanceDataBuffer["currentConductances"] = self.conductanceDataBuffer["currentConductances"][1:]
            else:
                self.conductanceDataBuffer["conductanceRaw"].append(conductanceRaw)

    def getRefValues(self, relativeTime):
        """returns reference voltages for standard soup based on currentRelativeTime"""
        referencepH = relativeTime ** 3 * (1.036e-12) + relativeTime ** 2 * (-1.358e-8) + relativeTime * (5.411e-5) + 1.8085
        if relativeTime < 300:
            referenceC = 4.8
        elif relativeTime < 900:
            referenceC = 3.4 - 0.2/600 * (relativeTime - 300)
        elif relativeTime < 960:
            referenceC = 3.2 - 0.7/60 * (relativeTime - 900)
        elif relativeTime < 1500:
            referenceC = 2.5 - 0.1/540 * (relativeTime - 960)

        return referencepH, referenceC

    def proportionalFeedbackTask(self):
        relativeTime = time.time() - self.cookingStartTime
        refpH, refC = self.getRefValues(relativeTime)
        Cvalues = self.conductanceDataBuffer["currentConductances"]
        Caverage = sum(Cvalues)/len(Cvalues)
        pHaverage = self.pHdataBuffer["currentpH"]
        Cdif = Caverage - refC
        pHdif = pHaverage - refpH

        waterSignal = 0.0
        saltSignal = 0.0
        if pHdif > 0:
            waterSignal = self.feedbackTask["kpH"] * pHdif / 4

        if Cdif > 0:
            saltSignal = self.feedbackTask["kC"] * Cdif / 6

        self.A.sendSignal(saltSignal, waterSignal)
        self.feedbackTask["startTime"] = relativeTime
        print("Doing Feedback \n|\n|\n|\n|\n|\n| pHdif = {a}, Cdif = {b}".format(a = pHdif, b = Cdif))
        print(saltSignal / 2, waterSignal * 10)

    def adjustEndRotation(self, targetAngle):
        try:
            self.UR.adjustEndRotation(targetAngle)
            print("moving to angle: {} degrees".format(targetAngle))
        except:
            print("error occured when trying to adjust angle, action abandoned!")
            pass

    def emergencyAction(self):
        """maybe not really needed for UR5 as you can just press the emergency button
        need to turn off water and salt dispenser"""
        print("------triggering emergency------")
        self.A.sendSignal(-100.0, -100.0) # Use (-10, -10) for emergency sign
        self.saveBufferToFile()

        while True:
            time.sleep(1)
            print("waiting to restart......")

    def step(self):
        """actions to be done in every main loop when cooking starts"""
        """record to dataBuffer"""
        relativeTime = time.time() - self.cookingStartTime
        distance, saltOn, waterOn = self.A.receiveSignal()
        pHRaw, TRaw, CRaw = self.D.receiveSignal()
        if distance > 0:
            self.dataBuffer.append((relativeTime, distance, saltOn, waterOn, pHRaw, TRaw, CRaw, self.conductanceTask["state"], self.pHdataBuffer["currentpH"], self.conductanceDataBuffer["currentConductances"][-1]))

        if self.enablePrint:
            print((relativeTime, distance, saltOn, waterOn, pHRaw, TRaw, CRaw, self.conductanceTask["state"]))
            print(self.pHdataBuffer["currentpH"], self.conductanceDataBuffer["currentConductances"][-1])

        self.measurementStep(pHRaw, CRaw, relativeTime)
        self.taskStep(relativeTime)
        time.sleep(0.05)
        self.counter += 1

    def menu(self):
        """activates when m key is pressed, will interrupt the current cooking processes"""

        keyboardInput = input("print: show/unshow data to be written in every step \n"
                         "load: start installing end actuator on UR5 \n"
                         "end: end cooking and terminate safely \n"
                         "height: move to targetHeight \n"
                         "heightUR: move relative height \n"
                         "heightDif: change conductance measurement up and down movement height"
                         "angle: rotate end to targetAngle \n"
                         "start: start cooking\n"
                         "resume: resume previous interrupted cooking\n"
                         "feedback: enable/disable feedback\n"
                         "conduct: do a conductance measurement\n"
                         "salt: add some salt\n"
                         "water: add some water\n"
                         "m: closes menu \n")

        if keyboardInput == "print":
            self.enablePrint = not self.enablePrint

        if keyboardInput == "feedback":
            self.enableFeedback = not self.enableFeedback

        if keyboardInput == "load":
            print("------moving to set up position------")
            self.UR.moveToSetupPosition()
            if input("move to ready position now? --y/n") == "y":
                self.UR.moveToReadyPosition()
                if input("startCooking now? --y/n") == "y":
                    self.startCooking()

        if keyboardInput == "start":
            self.startCooking()

        if keyboardInput == "height":
            try:
                targetHeight = float(input("input targetHeight in cm:"))
                self.adjustHeight(targetHeight)
            except:
                print("invalid input")
                pass

        if keyboardInput == "heightUR":
            try:
                relativeMovement = float(input("input height change in cm:"))
                self.UR.adjustToolPosition([0,0,relativeMovement/100])
            except:
                print("action denied")
                pass

        if keyboardInput == "heightDif":
            try:
                heightDif = float(input("input height difference in cm:"))
                self.conductanceTask["height"] = heightDif

            except:
                print("action denied")
                pass

        if keyboardInput == "angle":
            try:
                targetAngle = float(input("input angle in degrees:"))
                self.adjustEndRotation(targetAngle)
            except:
                print("invalid input")
                pass

        if keyboardInput == "end":
            self.endCooking()

        if keyboardInput == "resume":
            self.resumeCooking()

        if keyboardInput == "salt":
            try:
                targetSalt = float(input("input salt to add in grams:"))
                self.A.sendSignal(targetSalt * 2, 0.0)
            except:
                print("invalid input")

        if keyboardInput == "water":
            try:
                targetWater = float(input("input water to add in mL:"))
                self.A.sendSignal(0.0, targetWater/10)
            except:
                print("invalid input")

        if keyboardInput == "conduct":
            self.startConductanceTask()

        if keyboardInput == "m":
            pass

    def main(self):
        if keyboard.is_pressed("q"):
            self.emergencyAction()
        if keyboard.is_pressed("m"):
            self.menu()
            self.A.clearSerialBuffer()

        if self.cooking:
            if len(self.dataBuffer) > 150:
                self.saveBufferToFile()

            self.step()
        else:
            time.sleep(0.1)
            print("waiting for further action")

if __name__ == '__main__':
    CRcontrol = cookingRobot()
    while True:
        CRcontrol.main()