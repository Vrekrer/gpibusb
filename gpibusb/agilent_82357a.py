import usb
import subprocess
import time

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
            time.sleep(5)
        elif dev.idProduct == DEVICE_ID_82357B_PREINIT:
            dev_dir = '/dev/bus/usb/%03d/%03d' %(dev.bus, dev.address)
            firmware = firmware_directory + 'measat_releaseX1.8.hex'
            fxload_params = ['/sbin/fxload', '-t', 'fx2', 
                             '-D', dev_dir, 
                             '-I', firmware]
            #add exceptions!
            subprocess.check_call(fxload_params)
            time.sleep(5)

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
            time.sleep(5)

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

