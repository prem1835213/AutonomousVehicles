from megapi import MegaPi


MFR = 10     # port for motor front right 10
MBL = 11     # port for motor back left 11
MBR = 2    # port for motor back right 2
MFL = 3    # port for motor front left 3


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left


    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) +
              " MBR: " + repr(MBR) +
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) +
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))

        C_FL = 1
        C_FR = 1
        C_BL = 1 # 1.047
        C_BR = 1 # 1.043
        self.bot.motorRun(self.mfl, C_FL * vfl)
        self.bot.motorRun(self.mfr, C_FR * vfr)
        self.bot.motorRun(self.mbl, C_BL * vbl)
        self.bot.motorRun(self.mbr, C_BR * vbr)


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        # fl fr bl br
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)


    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )

    def close(self):
        self.bot.close()
        self.bot.exit()


if __name__ == "__main__":
    import time
    # calibration done on wood floor
    # rotation: motor speed 40, 5.4 secs for 1 full rotation
    # forward: motor speed 40, 5.4 secs for 3 feet
    # slide: motor speed 40, not calibrated due, aim for rotation and forward
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)
    time.sleep(3)
    mpi_ctrl.carStraight(40)
    time.sleep(3)
    mpi_ctrl.carSlide(40)
    time.sleep(3)
    time.sleep(5.4)
    # mpi_ctrl.carRotate(40)
    # time.sleep(5.4)
    mpi_ctrl.carStop()
    # print("If your program cannot be closed properly, check updated instructions in google doc.")
