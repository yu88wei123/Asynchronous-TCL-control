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

#freq(240000000)


def randstr():
    ranStr=str(random.randint(1,100000000))
    return ranStr

#---以下的参数值都需要根据自己的环境修改-----------------------------------------------
led=Pin(2,Pin.OUT) #ESP32的引脚2接了LED灯，可根据自己的ESP32板子的LED引脚来设置
spi = SoftSPI(sck=Pin(17), mosi=Pin(16), miso=Pin(5))
oled = SSD1306_SPI(width=128, height=64, spi=spi, dc=Pin(15),
                  res=Pin(4), cs=Pin(22))  
led.value(0)
#i2c=SoftI2C(scl=Pin(4), sda=Pin(15), freq=400000)
#oled = SSD1306_I2C(width=128, height=64, i2c=i2c)
oled.fill(0)

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

N_TCL = 0
N_MAX = 20
N_on = 0
SELF_ID = 16 * P34.value() + 8 * P35.value() + 4 * P32.value() + 2 * P33.value() + 1 * P25.value()

SSID = "ZTE_E3F7F2"  #填写自己的WIFI名称
PASSWORD = "yu88wei123"   #填写自己的WIFI密码

SERVER = '192.168.0.100'  # mqttHostUrl
CLIENT_ID = "MQTT_ESP32"+ randstr()  # clientId
username = 'yu88wei123' #username
password = 'yzw12345'  #密码

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

#至多三个邻居
#publish_TOPIC1 = 'test'+str(SELF_ID)+str(neighbor[0])
#publish_TOPIC2 = 'test'+str(SELF_ID)+str(neighbor[1])
#publish_TOPIC3 = 'test12' 
#subscribe_TOPIC1 = 'test'+str(neighbor[0])+str(SELF_ID)
#subscribe_TOPIC2 = 'test'+str(neighbor[1])+str(SELF_ID)

#特定的节点可以收到聚合商的信息
subscribe_TOPIC_AGG='AGG'


#subscribe_TOPIC3 = 'test1'
#---以上的参数值都需要根据自己的环境修改-----------------------------------------------

sendbuff = [0 for i in range(len(neighbor))]
sendflag = [0 for i in range(len(neighbor))]
receiveflag = [0 for i in range(len(neighbor))]
receivebuff = [0 for i in range(len(neighbor))]
client = None
mydht = None
wlan = None
STEP = 0 #当前的运行步骤
priority = 0 #优先级
prioied = 0 #是否已经完成优先级计算
inlist = 0
listposition = 0

Time_count = 0 #计时器计数
Time_step1 = 0
Time_step3 = 0
Time_step4 = 0
Time_step5 = 0
Time_step6 = 0
state_step4 = None
state_step5 = None
stepsize = 0.25  
switch_onoff = 0
decided = 0

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
print((b*T_out-c)/(1-a))

def ConnectWifi(ssid, passwd):
        global wlan
        wlan = network.WLAN(network.STA_IF)  # create a wlan object
        wlan.active(True)  # Activate the network interface
        wlan.disconnect()  # Disconnect the last connected WiFi
        wlan.connect(ssid, passwd)  # connect wifi
        while (wlan.ifconfig()[0] == '0.0.0.0'):
                time.sleep(1)
        print(wlan.ifconfig())

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
        #print((topic, msg))
        #msg = str(msg)
        #print(type(msg))
        #print(msg)
        msg = json.loads(msg)
        #print(topic,msg)
        #print(msg["state"])
            
        #第一阶段，判断是否收到start指令，并且是否为邻居发送的数据。如果是，则将指令广播给其他的节点
        ########################################################第一阶段#############################################################
        if SELF_ID == msg['sendto'] and msg['command'] =='start' and STEP == 0:
            if msg['sendfrom'] == 'aggregator' or msg['sendfrom'] in neighbor:
                STEP=1
                N_on = msg['N_on']
                N_TCL = msg['N_TCL']
                state_step4 = [0 for i in range(N_on)]
                state_step5 = [0 for i in range(N_on)]
                Time_step1 = Time_count
                
        #第三阶段，如果还没有进入第四阶段就收到了优先级序列的排序，立刻进入第四阶段并开始排序。除此之外则按照正常流程排序
        ########################################################第三阶段#############################################################
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
        #第四阶段，更新状态
        ########################################################第四阶段#############################################################
        elif STEP==4 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 4:
            state_step4_temp = state_step4
            state_step4.extend(msg['state']) #拼接两个数组
            state_step4 = list(set(state_step4)) #降序排序并剔除重复元素
            state_step4.sort(reverse=True)
            if len(state_step4) < N_on:
                for i in range(len(state_step4), N_on):  # 不够的位置补0
                    state_step4.append(0)
            if len(state_step4) > N_on:
                state_step4 = state_step4[:N_on]
        #第四阶段，如果还没有进入第五阶段就收到了优先级排序的指令，立刻进入第五阶段并开始排序。除此之外则按照正常流程排序
        ########################################################第四阶段#############################################################
        elif STEP==4 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 5:
            STEP = 5
            if priority in state_step4:
                    inlist = 1
                    listposition = state_step4.index(priority)
                    state_step5[listposition] = N_TCL
            Time_step5 = Time_count 
        #第五阶段，求均值
        ########################################################第四阶段#############################################################
        elif STEP==5 and msg['sendfrom'] in neighbor and msg['sendto'] == SELF_ID and msg['step'] == 5:
            if SELF_ID < msg['sendfrom'] and msg['command'] == 'ask':              # 确定是询问
                posi = neighbor.index(msg['sendfrom'])
                receivebuff[posi] = state_step5                     #将当前状态保存，用于发送
                receiveflag[posi] = 1
                list_3 = []
                for index, item in enumerate(msg['state']):         #收到询问立刻更新状态
                    list_3.append( state_step5[index] + stepsize * (item - state_step5[index]) )
                state_step5 = list_3
            if SELF_ID > msg['sendfrom'] and msg['command'] == 'reply':          # 确定是回复
                posi = neighbor.index(msg['sendfrom'])
                list_3 = []
                if sendflag[posi] == 1:
                    for index, item in enumerate(msg['state']):         #收到询问立刻更新状态
                        list_3.append( state_step5[index] + stepsize * (item - sendbuff[posi][index]) )
                    state_step5 = list_3
                    sendflag[posi] = 0
                

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
        
        #第二阶段，判断是否收到start指令，并且是否为邻居发送的数据。如果是，则将指令广播给其他的节点
        ########################################################第二阶段#############################################################
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
              
        #第三阶段，计算自身的优先级
        ########################################################第三阶段#############################################################
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
                

        #第四阶段，优先级序列计算
        ########################################################第四阶段#############################################################
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
          
          #第五阶段，优先级排序
        ########################################################第五阶段#############################################################
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
                if Time_count - Time_step5 > 250:                                                        
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
        #第六阶段，判断是否需要开启
        ########################################################第六阶段############################################################# 
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
        
#         try:
#                 mymessage = '{"heartbeat":"Device1"}'
#                 print('============================')
#                 print(mymessage)
#                 client.publish(topic=publish_TOPIC, msg=mymessage, retain=False, qos=0)
#         except Exception as ex_results2:
#                 print('exception', ex_results2)
#                 print('this is error')
#                 mytimer.deinit()
#     finally:
#         machine.reset()

                    

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
                
                client.set_callback(sub_cb)  # set callback # mqtt接收中断 多个topic对应一个中断，内部判断是谁发的
                
                client.connect()  # connect mqtt
                for i in range(len(neighbor)):
                    subscribe_TOPIC = 'test' +str(neighbor[i])+str(SELF_ID)
                    client.subscribe(subscribe_TOPIC)  # 订阅topic
                client.subscribe(subscribe_TOPIC_AGG)  # 订阅topic
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
