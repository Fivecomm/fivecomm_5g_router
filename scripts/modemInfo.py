#!/usr/bin/env python

# Copyright (c) 2022, Fivecomm - 5G COMMUNICATIONS FOR FUTURE INDUSTRY VERTICALS S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Description: File to extract Fivecomm modem information in ROS 
# Author: Pablo Trelis <pablo.trelis@fivecomm.eu> 
# Date: 2022/10/06
# Version: 1.0

# import libraries
from pexpect import pxssh
from datetime import datetime
import rospy
import time
import threading 
# ROS imports
from fivecomm_5g_router.msg import modemInfo, threegInfo, fourgInfo, fivegInfo, techInfo
from std_msgs.msg import Header
# Config VARS
enable_prints = False
frame_id = ''
modem_gateway = rospy.get_param("modemGateway")
modem_username = rospy.get_param("modemUsername")
modem_password = rospy.get_param("modemPassword")
pub_freq = rospy.get_param("pubFreq")
# Init VARS
tech = tech2 = mcc = mnc = cellid = rsrq5gnsa = sinr5gnsa = band5gnsa = rsrp4g = rsrq4g = rssi4g = ""
sinr4g = pcid = band4g = rsrp5g = rsrq5g = sinr5g = band5g = rscp3g = ecio3g = psc3g = band3g = ""
# ROS init
rospy.init_node('fivecomm_5g_router') 
modem_publish = rospy.Publisher('/fivecomm_5g_router/info', modemInfo, queue_size=100)

def getDataFromModem():
    global tech, tech2, mcc, mnc, cellid, rsrq5gnsa, sinr5gnsa, band5gnsa, rsrp4g, rsrq4g, rssi4g
    global sinr4g, pcid, band4g, rsrp5g, rsrq5g, sinr5g, band5g, rscp3g, ecio3g, psc3g, band3g
    s = pxssh.pxssh()
    try:
        s.login(modem_gateway, modem_username, modem_password)
    except:
        print("SSH session failed on login")
    else:
        print("SSH session login successful")
        th_2.start()
        while 0==0:
            s.sendline("varres=$(echo 'AT+QENG=\"servingcell\"'"
                        " | socat - /dev/ttyUSB2,crnl)")
            s.prompt()         # match the prompt
            s.sendline("echo $varres | awk -F'\"' '{print $8}'")
            s.prompt()
            tech = s.before.decode(encoding='UTF-8').split("\n")[1]
            s.sendline("prueba=$(echo $varres | awk -F\"QENG:\" '{print $4}')")
            s.prompt()
            s.sendline("echo $prueba | awk -F'\"' '{print $2}'")
            s.prompt()
            tech2 = s.before.decode(encoding='UTF-8').split("\n")[1]
            if("LTE" in tech):
                # MCC
                s.sendline("echo $varres | awk -F, '{print $4}'")
                s.prompt()
                mcc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # MNC
                s.sendline("echo $varres | awk -F, '{print $5}'")
                s.prompt()
                mnc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # CELLID
                s.sendline("echo $varres | awk -F, '{print $6}'")
                s.prompt()
                cellid = s.before.decode(encoding='UTF-8').split("\n")[1]
                # ----- 5G NSA -----
                if("NR5G-NSA" in tech2):
                    # RSRQ 5GNSA
                    s.sendline("echo $varres | awk -F, '{print $24}'")
                    s.prompt()
                    rsrq5gnsa = s.before.decode(encoding='UTF-8').split("\n")[1]
                    # SINR 5GNSA
                    s.sendline("echo $varres | awk -F, '{print $25}'")
                    s.prompt()
                    sinr5gnsa = s.before.decode(encoding='UTF-8').split("\n")[1]
                    # BAND 5GNSA
                    s.sendline("echo $varres | awk -F, '{print $27}'")
                    s.prompt()
                    band5gnsa = s.before.decode(encoding='UTF-8').split("\n")[1]
                # RSRP 4G
                s.sendline("echo $varres | awk -F, '{print $13}'")
                s.prompt()
                rsrp4g = s.before.decode(encoding='UTF-8').split("\n")[1]
                # RSRQ 4G
                s.sendline("echo $varres | awk -F, '{print $14}'")
                s.prompt()
                rsrq4g = s.before.decode(encoding='UTF-8').split("\n")[1]
                # RSSI 4G
                s.sendline("echo $varres | awk -F, '{print $15}'")
                s.prompt()
                rssi4g = s.before.decode(encoding='UTF-8').split("\n")[1]
                # SINR 4G
                s.sendline("echo $varres | awk -F, '{print $16}'")
                s.prompt()
                sinr4g = s.before.decode(encoding='UTF-8').split("\n")[1]
                # PCID 4G
                s.sendline("echo $varres | awk -F, '{print $7}'")
                s.prompt()
                pcid = s.before.decode(encoding='UTF-8').split("\n")[1]
                # BAND 4G
                s.sendline("echo $varres | awk -F, '{print $9}'")
                s.prompt()
                band4g = s.before.decode(encoding='UTF-8').split("\n")[1]
            elif("NR5G-SA" in tech):
                # MCC
                s.sendline("echo $varres | awk -F, '{print $5}'")
                s.prompt()
                mcc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # MNC
                s.sendline("echo $varres | awk -F, '{print $6}'")
                s.prompt()
                mnc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # CELLID
                s.sendline("echo $varres | awk -F, '{print $7}'")
                s.prompt()
                cellid = s.before.decode(encoding='UTF-8').split("\n")[1]
                # RSRP 5G
                s.sendline("echo $varres | awk -F, '{print $13}'")
                s.prompt()
                rsrp5g = s.before.decode(encoding='UTF-8').split("\n")[1] 
                # RSRQ 5G 
                s.sendline("echo $varres | awk -F, '{print $14}'")
                s.prompt()
                rsrq5g = s.before.decode(encoding='UTF-8').split("\n")[1]     
                # SINR 5G
                s.sendline("echo $varres | awk -F, '{print $15}'")
                s.prompt()
                sinr5g = s.before.decode(encoding='UTF-8').split("\n")[1]          
                # BAND 5G
                s.sendline("echo $varres | awk -F, '{print $11}'")
                s.prompt()
                band5g = s.before.decode(encoding='UTF-8').split("\n")[1]
            elif("WCDMA" in tech):
                # MCC
                s.sendline("echo $varres | awk -F, '{print $4}'")
                s.prompt()
                mcc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # MNC
                s.sendline("echo $varres | awk -F, '{print $5}'")
                s.prompt()
                mnc = s.before.decode(encoding='UTF-8').split("\n")[1]
                # CELLID
                s.sendline("echo $varres | awk -F, '{print $7}'")
                s.prompt()
                cellid = s.before.decode(encoding='UTF-8').split("\n")[1]
                # RSCP 3G
                s.sendline("echo $varres | awk -F, '{print $11}'")
                s.prompt()
                rscp3g = s.before.decode(encoding='UTF-8').split("\n")[1] 
                # ECIO 3G 
                s.sendline("echo $varres | awk -F, '{print $12}'")
                s.prompt()
                ecio3g = s.before.decode(encoding='UTF-8').split("\n")[1]     
                # PSC 3G
                s.sendline("echo $varres | awk -F, '{print $9}'")
                s.prompt()
                psc3g = s.before.decode(encoding='UTF-8').split("\n")[1]          
                # BAND 3G
                s.sendline("echo $varres | awk -F, '{print $8}'")
                s.prompt()
                band3g = s.before.decode(encoding='UTF-8').split("\n")[1]
            #time.sleep(0.5)
        s.logout()

def publishData():
    global tech, tech2, mcc, mnc, cellid, rsrq5gnsa, sinr5gnsa, band5gnsa, rsrp4g, rsrq4g, rssi4g
    global sinr4g, pcid, band4g, rsrp5g, rsrq5g, sinr5g, band5g, rscp3g, ecio3g, psc3g, band3g
    global modemInfo, modem_publish, frame_id
    while 0==0:
        current_time = str(datetime.now().time())[0:10]
        pub = modemInfo()
        info3g = threegInfo()
        info4g = fourgInfo()
        info5g = fivegInfo()
        infotech = techInfo()
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame_id
        pub.header = header
        pub.time = str(current_time)
        infotech.tech = str(tech.split("\r")[0])
        infotech.tech2 = str(tech2.split("\r")[0])
        infotech.mcc = str(mcc.split("\r")[0])
        infotech.mnc = str(mnc.split("\r")[0])
        infotech.cellid = str(cellid.split("\r")[0])
        pub.infotech = infotech
        info3g.rscp3g = str(rscp3g.split("\r")[0])
        info3g.ecio3g = str(ecio3g.split("\r")[0])
        info3g.psc3g = str(psc3g.split("\r")[0])
        info3g.band3g = str(band3g.split("\r")[0])
        pub.info3g = info3g
        info4g.rsrp4g = str(rsrp4g.split("\r")[0])
        info4g.rsrq4g = str(rsrq4g.split("\r")[0])
        info4g.rssi4g = str(rssi4g.split("\r")[0])
        info4g.sinr4g = str(sinr4g.split("\r")[0])
        info4g.pcid = str(pcid.split("\r")[0])
        info4g.band4g = str(band4g.split("\r")[0])
        pub.info4g = info4g
        info5g.rsrq5gnsa = str(rsrq5gnsa.split("\r")[0])
        info5g.sinr5gnsa = str(sinr5gnsa.split("\r")[0])
        info5g.band5gnsa = str(band5gnsa.split("\r")[0])
        info5g.rsrp5g = str(rsrp5g.split("\r")[0])
        info5g.rsrq5g = str(rsrq5g.split("\r")[0])
        info5g.sinr5g = str(sinr5g.split("\r")[0])
        info5g.band5g = str(band5g.split("\r")[0])
        pub.info5g = info5g
        modem_publish.publish(pub)
        if("LTE" in tech):
            if enable_prints:
                print("tech = " + tech)
                print("tech2 = " + tech2)
                print("mcc = " + mcc)
                print("mnc = " + mnc)
                print("cellid = " + cellid)
                print("rsrq5gnsa = " + rsrq5gnsa)
                print("sinr5gnsa = " + sinr5gnsa)
                print("band5gnsa = " + band5gnsa)
                print("rsrp4g = " + rsrp4g)
                print("rsrq4g = " + rsrq4g)
                print("rssi4g = " + rssi4g)
                print("sinr4g = " + sinr4g)
                print("pcid = " + pcid)
                print("band4g = " + band4g)
                print("======================================= " + current_time)
        elif("NR5G-SA" in tech):
            if enable_prints:
                print("tech = " + tech)
                print("mcc = " + mcc)
                print("mnc = " + mnc)
                print("cellid = " + cellid)
                print("rsrp5g = " + rsrp5g)
                print("rsrq5g = " + rsrq5g)
                print("sinr5g = " + sinr5g)
                print("band5g = " + band5g)
                print("======================================= " + current_time)  
        elif("WCDMA" in tech):
            None
        time.sleep(pub_freq)

if __name__ == '__main__':
    th_1 = threading.Thread(target = getDataFromModem)
    th_2 = threading.Thread(target = publishData)
    th_1.start()