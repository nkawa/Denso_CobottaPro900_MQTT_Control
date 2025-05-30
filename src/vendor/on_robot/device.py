#!/usr/bin/env python3
'''
Modified from https://github.com/cwsfa/onrobot-api/blob/master/scripts/device.py at commit id: da1eb38
'''

import xmlrpc.client

class Device:
    '''
    Generic device object
    '''
    cb = None

    def __init__(self, Global_cbip='192.168.1.1'):
        #try to get Computebox IP address
        try:
            self.Global_cbip = Global_cbip
        except NameError:
            print("Global_cbip is not defined!")

    def getCB(self):
            try:
                self.cb = xmlrpc.client.ServerProxy(
                    "http://" + str(self.Global_cbip) + ":41414/")
                return self.cb
            except TimeoutError:
                print("Connection to ComputeBox failed!")

if __name__ == '__main__':
    device = Device(Global_cbip='192.168.5.46')
    cb = device.getCB()
    for m in cb.system.listMethods():
        print(m, cb.system.methodSignature(m))
