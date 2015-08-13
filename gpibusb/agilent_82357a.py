import usb
import subprocess
import time
import enum

#detect conected AGILENT_82357 USB devices and load the firmware on them

firmware_directory = '/usr/share/usb/agilent_82357a/'

def load_firmawe_on_devices():
    VENDOR_ID_AGILENT = 0x0957

    #device ids before firmware is loaded 
    DEVICE_ID_82357A_PREINIT = 0x0007
    DEVICE_ID_82357B_PREINIT = 0x0518
    dev_it = usb.core.find(find_all=True, idVendor = VENDOR_ID_AGILENT)
    for dev in dev_it:
        if dev.idProduct == DEVICE_ID_82357A_PREINIT:
            dev_dir = '/dev/bus/usb/%03d/%03d' %(dev.bus, dev.address)
            firmware = firmware_directory + '82357a_fw.hex'
            fxload_params = ['/sbin/fxload',
                             '-D', dev_dir, 
                             '-I', firmware]
            #add exceptions!
            subprocess.check_call(fxload_params)
            time.sleep(3)
        elif dev.idProduct == DEVICE_ID_82357B_PREINIT:
            dev_dir = '/dev/bus/usb/%03d/%03d' %(dev.bus, dev.address)
            firmware = firmware_directory + 'measat_releaseX1.8.hex'
            fxload_params = ['/sbin/fxload', '-t', 'fx2', 
                             '-D', dev_dir, 
                             '-I', firmware]
            #add exceptions!
            subprocess.check_call(fxload_params)
            time.sleep(3)

    dev_it = usb.core.find(find_all=True, idVendor = VENDOR_ID_AGILENT)
    #run again for 82357B devices
    for dev in dev_it:
        if dev.idProduct == DEVICE_ID_82357B_PREINIT:
            dev_dir = '/dev/bus/usb/%03d/%03d' %(dev.bus, dev.address)
            firmware = firmware_directory + 'measat_releaseX1.8.hex'
            fxload_params = ['/sbin/fxload', '-t', 'fx2', 
                             '-D', dev_dir, 
                             '-I', firmware]
            #add exceptions!
            subprocess.check_call(fxload_params)
            time.sleep(3)

def get_devices():
    VENDOR_ID_AGILENT = 0x0957
    #device ids with firmware loaded 
    DEVICE_ID_82357A = 0x0107
    DEVICE_ID_82357B = 0x0718

    #get the firmware loaded devices
    dev_it = usb.core.find(find_all=True, idVendor = VENDOR_ID_AGILENT)
    valid_IDs = [DEVICE_ID_82357A, DEVICE_ID_82357B]
    return [dev for dev in dev_it if dev.idProduct in valid_IDs]    

load_firmawe_on_devices()
devices = get_devices()

#TMS9914 Register Addresses
#Read registers
REG_INTERRUPT_STATUS_0 = 0b000
REG_INTERRUPT_STATUS_1 = 0b001
REG_ADDRESS_STATUS     = 0b010
REG_BUS_STATUS         = 0b011
REG_COMMAND_PASS_THRU  = 0b110
REG_DATA_IN            = 0b110
#Write registers
REG_INTERRUPT_MASK_0  = 0b000
REG_INTERRUPT_MASK_1  = 0b001
REG_AUXILIARY_COMMAND = 0b011
REG_ADDRESS           = 0b100
REG_SERIAL_POLL       = 0b101
REG_PARALLEL_POLL     = 0b110
REG_DATA_OUT          = 0b111
#Auxiliary commands
AUXC_RHDF = 0x02 #Release RFD holdoff
AUXC_NBAF = 0x05 #New byte aviable false
AUXC_FEOI = 0x08 #Send EOI with next byte
AUXC_GTS  = 0x0B #Go to standby
AUXC_TCA  = 0x0C #Take control asynchronously
AUXC_TCS  = 0x0D #Take control synchronously
AUXC_RQC  = 0x11 #Request Control
AUXC_RLC  = 0x12 #Release Control
AUXC_PTS  = 0x14 #Pass through next secondary
AUXC_CLEAR_SWRST = 0x00 #Software reset
AUXC_SET_SWRST   = 0x80
AUXC_CLEAR_DACR  = 0x01 #Release DAC holdoff
AUXC_SET_DACR    = 0x81
AUXC_CLEAR_HDFA  = 0x03 #Hold on all data
AUXC_SET_HDFA    = 0x83
AUXC_CLEAR_HDFE  = 0x04 #Hold on EOI only
AUXC_SET_HDFA    = 0x84
AUXC_CLEAR_FGET  = 0x06 #Force group execute trigger
AUXC_SET_FGET    = 0x86
AUXC_CLEAR_RTL   = 0x07 #Return to local
AUXC_SET_RTL     = 0x97
AUXC_CLEAR_LON   = 0x09 #Lisen only
AUXC_SET_LON     = 0x89 
AUXC_CLEAR_TON   = 0x0A #Talk only
AUXC_SET_TON     = 0x8A
AUXC_CLEAR_RPP   = 0x0E #Request parallel poll
AUXC_SET_RPP     = 0x8E
AUXC_CLEAR_SIC   = 0x0F #Send interface clear
AUXC_SET_SIC     = 0x8F
AUXC_CLEAR_SRE   = 0x10 #Send remote enable
AUXC_SET_SRE     = 0x90
AUXC_CLEAR_DAI   = 0x13 #Disable all interrupts
AUXC_SET_DAI     = 0x93
AUXC_CLEAR_STD1  = 0x15 #Short T1 settling time
AUXC_SET_STD1    = 0x95
AUXC_CLEAR_SHDW  = 0x16 #Shadow handshake
AUXC_SET_SHDW    = 0x86
AUXC_CLEAR_VSTD1 = 0x17 #Very short T1 delay
AUXC_SET_VSTD1   = 0x97
AUXC_CLEAR_RSV2  = 0x18 #Request service bit 2
AUXC_SET_RSV2    = 0x98


#Firmware Regiters
REG_HW_CONTROL       = 0xa
REG_LED_CONTROL      = 0xb
REG_RESET_TO_POWERUP = 0xc
REG_PROTOCOL_CONTROL = 0xd
REG_FAST_TALKER_T1   = 0xe
#Hardware Control Bits
HW_NOT_TI_RESET      = 0b001
HW_SYSTEM_CONTROLLER = 0b010
HW_NOT_PARALLEL_POLL = 0b100
#LED Control Bits
LED_FIRMWARE_CONTROL = 0x1
LED_FAIL_ON          = 0x20
LED_READY_ON         = 0x40
LED_ACCESS_ON        = 0x80
#Reset to powerup Bits
RESET_SPACEBALL = 0x1   # wait 2 millisec after sending
#Protocol control bits
PROTOCOL_WRITE_COMPLETE_INTERRUPT_EN = 0x1

#Error codes
ERR_SUCCESS          = 0 #No error
ERR_INVALID_CMD      = 1
ERR_INVALID_PARAM    = 2
ERR_INVALID_REG      = 3
ERR_GPIB_READ        = 4
ERR_GPIB_WRITE       = 5
ERR_FLUSHING         = 6
ERR_FLUSHING_ALREADY = 7    
ERR_UNSUPPORTED      = 8
ERR_OTHER            = 9

class _BusStatus(object):
    def __init__(self, statusByte):
        self.ATN  = bool(statusByte & 0b10000000)
        self.DAV  = bool(statusByte & 0b01000000)
        self.NDAC = bool(statusByte & 0b00100000)
        self.NRFD = bool(statusByte & 0b00010000)
        self.EOI  = bool(statusByte & 0b00001000)
        self.SRQ  = bool(statusByte & 0b00000100)
        self.IFC  = bool(statusByte & 0b00000010)
        self.REN  = bool(statusByte & 0b00000001)
    def __repr__(self):
        lines = ['EOI', 'DAV', 'NRFD', 'NDAC',
                 'IFC', 'SRQ', 'ATN', 'REN']
        rep = ['GPIB Bus lines status:']
        for l in lines:
            rep.append(' %4s = %s' %(l, self.__dict__[l]) )
        return '\n'.join(rep)
        

class Agilent_82357_Device(object):
    def __init__(self, USB_device):
        self.device = USB_device
        self.device.reset()
        self.device.set_configuration()
        self._dev_cfg = self.device.get_active_configuration()
        self._dev_cfg_intf = self._dev_cfg[(0,0)]
        self._endpoints = {}
        for ep in self._dev_cfg_intf:
            if 'Bulk IN' in ep.__repr__():
                self._endpoints['Bulk IN'] = ep
            elif 'Bulk OUT' in ep.__repr__():
                self._endpoints['Bulk OUT'] = ep
            elif 'Interrupt IN' in ep.__repr__():
                self._endpoints['Interrupt'] = ep

        self._writeRegisters([(REG_RESET_TO_POWERUP, RESET_SPACEBALL)])
        time.sleep(0.002) #2ms delay after RESET_SPACEBALL

        #clear all unwanted features of TMS9914
        #set the T1 delay
        #configure the address
        #enable interrupts
        #set interrupt masks
        #end configuration by clearing sofware reset
        #let the firmawere control de leds
        REG_AUX = REG_AUXILIARY_COMMAND
        init_pairs = [(REG_AUX, AUXC_CLEAR_DACR),
                      (REG_AUX, AUXC_CLEAR_HDFA),
                      (REG_AUX, AUXC_CLEAR_HDFE),
                      (REG_AUX, AUXC_CLEAR_LON),
                      (REG_AUX, AUXC_CLEAR_TON),
                      (REG_AUX, AUXC_CLEAR_STD1),
                      (REG_AUX, AUXC_CLEAR_VSTD1),
                      (REG_AUX, AUXC_CLEAR_RSV2),
                      (REG_FAST_TALKER_T1, 0x26), #798 ns
                      (REG_ADDRESS, 0x00), #0x00 for controler?
                      (REG_PROTOCOL_CONTROL, 
                              PROTOCOL_WRITE_COMPLETE_INTERRUPT_EN),
                      (REG_INTERRUPT_MASK_0, 0b00110000),
                      (REG_INTERRUPT_MASK_1, 0b00000010),
                      (REG_AUX, AUXC_CLEAR_SWRST),
                      (REG_LED_CONTROL, LED_FIRMWARE_CONTROL)
                     ]
        self._writeRegisters(init_pairs)

    def _writeRegisters(self, pairs_reg_data, timeout = 100):
        #timeout in ms
        WRITE_REGS = 0x04
        header = [WRITE_REGS, len(pairs_reg_data)]
        unpacked_pairs = list(sum(pairs_reg_data, ()))
        data = header + unpacked_pairs
        self._endpoints['Bulk OUT'].write(data, timeout)
        response = self._endpoints['Bulk IN'].read(32, timeout)
        if response[0] != 0xFF - WRITE_REGS:
            #raise error here
            print('Bad ~WRITE_REGS')
        if response[1] != 0:
            #raise error here
            print('Error code ', response[1])
            
    def _readRegisters(self, registers, timeout = 100):
        #timeout in ms
        READ_REGS = 0x05
        header = [READ_REGS, len(registers)]
        data = header + list(registers)
        self._endpoints['Bulk OUT'].write(data, timeout)
        response = self._endpoints['Bulk IN'].read(32, timeout)
        if response[0] != 0xFF - READ_REGS:
            #raise error here
            print('Bad ~READ_REGS', response[0])
        if response[1] != 0:
            #raise error here
            print('Error code ', response[1])
        return list(response[2:])

    @property
    def lineStatus(self):
        statusByte = self._readRegisters([REG_BUS_STATUS])[0]
        return _BusStatus(statusByte)

    def requestSystemControl(self, control):
        HW_C_bits = self._readRegisters([REG_HW_CONTROL])[0]
        regs = [REG_AUXILIARY_COMMAND, REG_HW_CONTROL]
        if control:
            vals = [AUXC_RQC, HW_C_bits | HW_SYSTEM_CONTROLLER]
        else:
            vals = [AUXC_RLC, HW_C_bits & (0xFF - HW_SYSTEM_CONTROLLER)]
        reg_pairs = zip(regs, vals)
        self._writeRegisters(reg_pairs)
    
    def takeControl(self, async = True):
        if async:
            TC_mode = AUXC_TCA
        else:
            TC_mode = AUXC_TCS
        self._writeRegisters([(REG_AUXILIARY_COMMAND, TC_mode)])

    def goToStandby(self):
        self._writeRegisters([(REG_AUXILIARY_COMMAND, AUXC_GTS)])
        
    @property
    def T1_delay(self):
        NANOSEC_PER_BIT = 21.0
        T1_byte = self._readRegisters([REG_FAST_TALKER_T1])[0]
        return T1_byte*NANOSEC_PER_BIT
    @T1_delay.setter
    def T1_delay(self, T1):
        NANOSEC_PER_BIT = 21.0
        MAX_VALUE = 0x72*0 + 255
        MIN_VALUE = 0x11*0
        T1_byte = int(T1/NANOSEC_PER_BIT + 0.5)
        T1_byte = max(T1_byte, MIN_VALUE)
        T1_byte = min(T1_byte, MAX_VALUE)
        self._writeRegisters([(REG_FAST_TALKER_T1, T1_byte)])
