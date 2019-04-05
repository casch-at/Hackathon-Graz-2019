#!/usr/bin/env python
from subprocess import check_output, CalledProcessError
import shlex

VENDOR_PRODUCT_ID = '0x1c28:0xc012'


def usb_cams():
    cmd = shlex.split('lsusb -d {}'.format(VENDOR_PRODUCT_ID))
    try:
        output = check_output(cmd).strip()
    except CalledProcessError:
        return ''

    lines = output.split('\n')
    if not lines:
        return ''

    lsstring = lines[0]
    data = lsstring.split(' ')
    bus = data[1]
    device = data[3][:-1]

    return '--device /dev/bus/usb/{}/{}'.format(bus, device)


print(usb_cams())
