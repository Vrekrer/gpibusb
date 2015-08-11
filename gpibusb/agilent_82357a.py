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

#Firmware Regiters
REG_HW_CONTROL       = 0xa
REG_LED_CONTROL      = 0xb
REG_RESET_TO_POWERUP = 0xc
REG_PROTOCOL_CONTROL = 0xd
REG_FAST_TALKER_T1   = 0xe
#Hardware Control Bits
HW_NOT_TI_RESET      = 0x1
HW_SYSTEM_CONTROLLER = 0x2
HW_NOT_PARALLEL_POLL = 0x4
HW_OSCILLATOR_5V_ON  = 0x8
HW_OUTPUT_5V_ON      = 0x20
HW_CPLD_3V_ON        = 0x80
#LED Control Bits
LED_FIRMWARE_CONTROL = 0x1
LED_FAIL_ON          = 0x20
LED_READY_ON         = 0x40
LED_ACCESS_ON        = 0x80
#Reset to powerup Bits
RESET_SPACEBALL = 0x1	# wait 2 millisec after sending
#Protocol control bits
PROTOCOL_WRITE_COMPLETE_INTERRUPT_EN = 0x1

class AGILENT_82357_Device(object):
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

        init_pairs = [(REG_LED_CONTROL, LED_FIRMWARE_CONTROL),
                      (REG_RESET_TO_POWERUP, RESET_SPACEBALL)
                     ]
        self._writeRegisters(init_pairs)
        time.sleep(0.1)

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

