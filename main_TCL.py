#-*-coding:utf-8-*-
from umqtt.simple import MQTTClient
from machine import Pin, SoftSPI, SoftI2C, freq
import network
import time
import machine
import dht
from machine import Timer
import json
from ssd1306 import SSD1306_SPI, SSD1306_I2C
import random
import math

# Set the MCU operation frequency, the default is 160MHz
#freq(240000000)


# Generate a random ID for connecting to the MQTT server
def randstr():
    ranStr=str(random.randint(1,100000000))
    return ranStr

#The following parameter values all need to be modified according to the specific MCU
#The following parameter values all need to be modified according to the specific MCU
#The following parameter values all need to be modified according to the specific MCU
led=Pin(2,Pin.OUT) #Initialize the on-board LED

#Initialize the OLED for displaying some variables for debugging purposes.
spi = SoftSPI(sck=Pin(17), mosi=Pin(16), miso=Pin(5))
oled = SSD1306_SPI(width=128, height=64, spi=spi, dc=Pin(15),
                  res=Pin(4), cs=Pin(22))  
led.value(0)
oled.fill(0)



# Initialize the input pins and set the ESP32's ID in the communication network by connecting these pins to VCC or GND.
P34=Pin(34)
P35=Pin(35)
P32=Pin(32)
P33=Pin(33)
P25=Pin(25)
P34.init(Pin.IN,Pin.PULL_DOWN)
P35.init(Pin.IN,Pin.PULL_DOWN)
P32.init(Pin.IN,Pin.PULL_DOWN)
P33.init(Pin.IN,Pin.PULL_DOWN)
P25.init(Pin.IN,Pin.PULL_DOWN)
SELF_ID = 16 * P34.value() + 8 * P35.value() + 4 * P32.value() + 2 * P33.value() + 1 * P25.value()



N_TCL = 0
N_on = 0
N_MAX = 20 # Maximum number of TCLs


# Parameters for connecting to WIFI and MQTT
SSID = "ZTE_E3F7F2"  # Fill in your own WIFI name
PASSWORD = "yu88wei123"   # Fill in your WIFI password
SERVER = '192.168.0.100'  # mqttHostUrl
CLIENT_ID = "MQTT_ESP32"+ randstr()  # clientId
username = 'yu88wei123' # username
password = 'yzw12345'  # password

# Setting up TCL's neighbors
if SELF_ID == N_MAX:
    neighbor = [1, 2, N_MAX-2, N_MAX-1] 
    neighbor=list(set(neighbor))
elif SELF_ID == N_MAX - 1:
    neighbor = [1, SELF_ID-2, SELF_ID-1, N_MAX]
    neighbor=list(set(neighbor))
elif SELF_ID == 1:
    neighbor = [2, 3, N_MAX-1, N_MAX]
    neighbor=list(set(neighbor))
elif SELF_ID == 2:
    neighbor = [1, SELF_ID+1, SELF_ID+2, N_MAX]
    neighbor=list(set(neighbor))
else:
    neighbor = [SELF_ID-2, SELF_ID-1, SELF_ID+1, SELF_ID+2] 
    neighbor=list(set(neighbor))

print(neighbor)

#Specific nodes can receive information from the aggregator
subscribe_TOPIC_AGG='AGG'


sendbuff = [0 for i in range(len(neighbor))]
sendflag = [0 for i in range(len(neighbor))]
receiveflag = [0 for i in range(len(neighbor))]
receivebuff = [0 for i in range(len(neighbor))]
client = None
mydht = None
wlan = None


STEP = 0 # Current operating steps
priority = 0 # priority
prioied = 0 # Whether the prioritization has been completed
inlist = 0 # Whether the priority is in the list
listposition = 0 # Priority position in the list

Time_count = 0 # timer count
Time_step1 = 0
Time_step3 = 0
Time_step4 = 0
Time_step5 = 0
Time_step6 = 0



state_step4 = None # S
state_step5 = None # Y
stepsize = 0.25   # epsilon
switch_onoff = 0 # operation state s
decided = 0 # Whether the entire control process is completed


# Initialize ETPs
C = 3.5 + 2*random.random()                    
R = 2 + 1.5*random.random()
theta = 3
T_max = 28
T_min = 24
P_rate = 2 
T_in = 23 + 6*random.random()
T_out = 32
Time_scale = 300
a = math.exp(-Time_scale /3600 / R / C)
b = 1 - a
c = b * theta * R * P_rate


# Connect Wifi
def ConnectWifi(ssid, passwd):
        global wlan
        wlan = network.WLAN(network.STA_IF)  # create a wlan object
        wlan.active(True)  # Activate the network interface
        wlan.disconnect()  # Disconnect the last connected WiFi
        wlan.connect(ssid, passwd)  # connect wifi
        while (wlan.ifconfig()[0] == '0.0.0.0'):
                time.sleep(1)
        print(wlan.ifconfig())


# Processing of received data
def sub_cb(topic, msg):
        global led
        global STEP
        global client
        global neighbor
        global N_MAX
        global N_TCL
        global N_on
        global SELF_ID
        global Time_count
        global Time_step1
        global Time_step3
        global Time_step4
        global Time_step5
        global Time_step6
        global priority
        global prioied
        global state_step4
        global state_step5
        global inlist
        global listposition
        global stepsize
        global sendbuff
        global sendflag
        global receiveflag
        global receivebuff
        global switch_onoff
        global decided
        global a
        global b
        global c
        global T_in

        msg = json.loads(msg)
            
        #In the first step, determine if the start command is received.
        ########################################################Step 1#############################################################
        if SELF_ID == msg['sendto'] and msg['command'] =='start' and STEP == 0:
            if msg['sendfrom'] == 'aggregator' or msg['sendfrom'] in neighbor:
                STEP=1
                N_on = msg['N_on']
                N_TCL = msg['N_TCL']
                state_step4 = [0 for i in range(N_on)]
                state_step5 = [0 for i in range(N_on)]
                Time_step1 = Time_count
                
        #In the third step,If the neighbor is already entered the fourth step, Move on to step 4 immediately.
        ########################################################Step 3#############################################################
        elif STEP==3 and prioied==1 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 4:
            STEP = 4
            prioied = 0
            i = 0
            T_pre = T_in
            while T_pre > T_min:
                i = i + 1
                T_pre = a * T_pre + b * T_out - c
            priority = i + random.randint(0,4) *0.2
            state_step4[0] = priority
            Time_step4 = Time_count
        #Step 4: Update Status
        ########################################################Step 4#############################################################
        elif STEP==4 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 4:
            state_step4_temp = state_step4
            state_step4.extend(msg['state'])
            state_step4 = list(set(state_step4)) #Sorting and eliminating duplicate elements
            state_step4.sort(reverse=True)
            if len(state_step4) < N_on:
                for i in range(len(state_step4), N_on):
                    state_step4.append(0)
            if len(state_step4) > N_on:
                state_step4 = state_step4[:N_on]
        #In the fourth step,If the neighbor is already entered the fifthly step, Move on to step 5 immediately.
        ########################################################Step 4#############################################################
        elif STEP==4 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 5:
            STEP = 5
            if priority in state_step4:
                    inlist = 1
                    listposition = state_step4.index(priority)
                    state_step5[listposition] = N_TCL
            Time_step5 = Time_count 
        #Step 5, finding the mean value
        ########################################################Step 5#############################################################
        elif STEP==5 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 5:
            if SELF_ID < msg['sendfrom'] and msg['command'] == 'ask':              # Determined to be an inquiry
                posi = neighbor.index(msg['sendfrom'])
                receivebuff[posi] = state_step5                     # Save the current state for sending
                receiveflag[posi] = 1
                list_3 = []
                for index, item in enumerate(msg['state']):         # Update state
                    list_3.append( state_step5[index] + stepsize * (item - state_step5[index]) )
                state_step5 = list_3
            if SELF_ID > msg['sendfrom'] and msg['command'] == 'reply':          # Determined to be an reply
                posi = neighbor.index(msg['sendfrom'])
                list_3 = []
                if sendflag[posi] == 1:
                    for index, item in enumerate(msg['state']):         # Update state
                        list_3.append( state_step5[index] + stepsize * (item - sendbuff[posi][index]) )
                    state_step5 = list_3
                    sendflag[posi] = 0
                
# periodic interrupt handler
def heartbeatTimer(mytimer):
        global client
        global led
        global N_MAX
        global N_TCL
        global N_on
        global neighbor
        global SELF_ID
        global STEP
        global Time_count
        global Time_step1
        global Time_step3
        global Time_step4
        global Time_step5
        global Time_step6
        global priority
        global prioied
        global state_step4
        global state_step5
        global inlist
        global listposition
        global stepsize
        global sendbuff
        global sendflag
        global receiveflag
        global receivebuff
        global switch_onoff
        global decided
        global a
        global b
        global c
        global T_in

#  LED blinking, used to determine whether the interrupt is entered normally or not
        if STEP>=1:
            light=1-led.value()
            led.value(light)
        
#         oled.fill(0)
#         #oled.text('SELF_ID  '+str(SELF_ID), 0, 10)
#         oled.text('SELFID '+str(SELF_ID)+' onoff '+str(switch_onoff), 0, 0)
#         oled.text('STEP '+str(STEP)+' prior '+str(priority), 0, 20)
#         oled.text('nei '+str(neighbor), 0, 30)
#         #oled.text('priority  '+str(priority), 0, 30)
#         if STEP>=1:
#             oled.text('NTCL '+str(N_TCL)+' Non '+str(N_on), 0, 10)
#             float_lst = []
#             for i in range(min(N_on,3)):
#                 float_lst.append(round(state_step5[i], 1))
#             oled.text('s4  '+str(state_step4[0:min(N_on,3)]), 0, 40)
#             oled.text('s5  '+str(float_lst), 0, 50)
#         oled.show()
        
        Time_count = Time_count + 1
        
        # Determines if the start command was received. If so, broadcast the command to the other nodes
        ########################################################Step 2#############################################################
        if STEP == 1:
            if Time_count - Time_step1 < 25:                    
                for nei in neighbor:
                    mymessage={"step":STEP,"sendfrom":SELF_ID,"sendto":nei,"command":"start","state":[],"N_on":N_on,"N_TCL":N_TCL}
                    mymessage = json.dumps(mymessage)
                    publish_TOPIC = 'test'+str(SELF_ID)+str(nei)
                    client.publish(topic=publish_TOPIC, msg=mymessage, retain=False, qos=0)
                    time.sleep_ms(30)
                    
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":[]}
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)
                    
                #STEP=3
            if Time_count - Time_step1 > 40:                                  
                STEP = 2
              
        # Calculating own priorities
        ########################################################Step 3#############################################################
        elif STEP==2:
            if Time_count - Time_step1 > 50:      
                STEP=3
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":[]}
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)
                
        elif STEP==3:                     
            if prioied == 0:
                i = 0
                T_pre = T_in
                while T_pre > T_min:
                    i = i + 1
                    T_pre = a * T_pre + b * T_out - c
                    if(i>100):
                        break
                priority = i + random.randint(0,4) *0.2                      
                prioied = 1
                Time_step3 = Time_count
                state_step4[0] = priority
            if Time_count - Time_step3 > 20:
                STEP = 4
                prioied = 0
                Time_step4 = Time_count
            
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":priority}
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)
                

        #Priority Sequence Acquisition
        ########################################################Step 4#############################################################
        elif STEP==4:
            for nei in neighbor:
                mymessage={"step":STEP,"sendfrom":SELF_ID,"sendto":nei,"command":"","state":state_step4,"N_on":N_on,"N_TCL":N_TCL}
                mymessage = json.dumps(mymessage)
                publish_TOPIC = 'test'+str(SELF_ID)+str(nei)
                client.publish(topic=publish_TOPIC, msg=mymessage, retain=False, qos=0)
                time.sleep_ms(30)
                
            if Time_count - Time_step4 > 75:                                   
                STEP = 5
                if priority in state_step4:
                    inlist = 1
                    listposition = state_step4.index(priority)
                    state_step5[listposition] = N_TCL
                Time_step5 = Time_count
                
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":state_step4}
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)
          
          #Priority Ordering
        ########################################################Step 5#############################################################
        elif STEP==5:
            for nei in neighbor:
                if SELF_ID > nei: #向ID更小的节点发送请求
                    posi = neighbor.index(nei)
                    if sendflag[posi] == 0:   #如果没有发送过请求或者已经完成回复
                        mymessage={"step":STEP,"sendfrom":SELF_ID,"sendto":nei,"command":"ask","state":state_step5,"N_on":N_on,"N_TCL":N_TCL} # ask代表询问指令
                        mymessage = json.dumps(mymessage)
                        publish_TOPIC = 'test'+str(SELF_ID)+str(nei)
                        client.publish(topic=publish_TOPIC, msg=mymessage, retain=False, qos=0)
                        sendbuff[posi] = state_step5
                        sendflag[posi] = 1
                        time.sleep_ms(30)
                if SELF_ID < nei:  #判断是否需要回复
                    posi = neighbor.index(nei)
                    if receiveflag[posi] == 1:
                        mymessage={"step":STEP,"sendfrom":SELF_ID,"sendto":nei,"command":"reply","state":receivebuff[posi][:],"N_on":N_on,"N_TCL":N_TCL} # ask代表询问指令
                        mymessage = json.dumps(mymessage)
                        publish_TOPIC = 'test'+str(SELF_ID)+str(nei)
                        client.publish(topic=publish_TOPIC, msg=mymessage, retain=False, qos=0)
                        receiveflag[posi] = 0   #回复完成
                        time.sleep_ms(30)
                if Time_count - Time_step5 > 150:                                                                            
                    STEP = 6
                    float_lst = []
                    for i in range(len(state_step5)):
                        float_lst.append(round(state_step5[i]))
                    state_step5 = float_lst
                    Time_step6 = Time_count
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":state_step5}
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)
        #Local Decision-Making
        ########################################################Step 6#############################################################
        elif STEP==6:   
            if decided==0:
                totalhigher = 0
                if inlist == 1:
                    for i in range(listposition):
                        totalhigher = totalhigher + state_step5[i]
                    if totalhigher >= N_on:
                        switch_onoff = 0
                    elif totalhigher + state_step5[listposition] <= N_on:
                        switch_onoff = 1
                    elif random.random() <= (N_on - totalhigher) / state_step5[listposition]:
                        switch_onoff = 1
                    else:
                        switch_onoff = 0
                else: 
                    switch_onoff = 0    
                decided = 1
                T_in = a * T_in + b * T_out - c * switch_onoff
            #if Time_count - Time_step6 > 20:
            STEP = 0
            sendbuff = [0 for i in range(len(neighbor))]
            sendflag = [0 for i in range(len(neighbor))]
            receiveflag = [0 for i in range(len(neighbor))]
            receivebuff = [0 for i in range(len(neighbor))]
                #priority = 0 
            prioied = 0
            inlist = 0
            listposition = 0
            state_step4 = None
            state_step5 = None
            decided = 0
            led.value(switch_onoff)
            mymessage1={"step":STEP,"sendfrom":SELF_ID,"sendto":'AGG',"state":switch_onoff,"temp":T_in} 
            mymessage1 = json.dumps(mymessage1)
            publish_TOPIC = 'test'+'AGG'+str(SELF_ID)
            client.publish(topic=publish_TOPIC, msg=mymessage1, retain=False, qos=0)


                    

# main function
def run():
        global client
        global led
        global wlan
        print('start to connect mqtt')
        try:
                #mydht = dht.DHT11(machine.Pin(4))
                ConnectWifi(SSID, PASSWORD)
                #print('client:%s' % str(client))
                client = MQTTClient(CLIENT_ID, SERVER, 1883) # create a mqtt client
                print('client:%s' % str(client))
                #led.value(1)
                
                client.set_callback(sub_cb)  # set callback
                
                client.connect()  # connect mqtt
                for i in range(len(neighbor)):
                    subscribe_TOPIC = 'test' +str(neighbor[i])+str(SELF_ID)
                    client.subscribe(subscribe_TOPIC)  # subscribe topic
                client.subscribe(subscribe_TOPIC_AGG)  # subscribe topic
                mytimer = Timer(0)
                mytimer.init(mode=Timer.PERIODIC, period=200, callback=heartbeatTimer)
                led.value(1)
                while True:
                        client.wait_msg()  # wait message

        except Exception  as ex_results:
                print('exception1', ex_results)
        finally:
                if (client is not None):
                        led.value(0)
                        client.disconnect()
                wlan.disconnect()
                wlan.active(False)
                return 'FAILED'
   
   


while (True):
    if run() == 'FAILED':
        print('FAILED,retry to connect')
        time.sleep(5)
