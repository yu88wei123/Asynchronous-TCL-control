# Asynchronous-TCL-control

The following code corresponds to the semi-physical experimental part of the paper 	**"Asynchronous Communication-Based Distributed Control Strategy for ON/OFF State Thermostatically Controlled Loads"**.

main_AGG.mlx is the PC's code that simulates the aggregator. The PC is only responsible for issuing the number of TCLs that need to be turned on in total during this control process.

main_TCL.py is the MCU code to simulate TCLs and implement asynchronous distributed communication.

The PC code needs to run on **Matlab2023a**.

Before running main_TCL.py, you need to install **Micropython** on your MCU. The MCU we chose is ESP32-WROOM-32. You can choose any other MCU, but please make sure your MCU supports micropython. In addition, you may need to modify the code for the specific MCU. The download link for micropython: https://micropython.org/download/?mcu=esp32

We have used WIFI+MQTT to build a communication network. There are corresponding network connection functions in the code. If you want to use the code we provided, you need to build an MQTT server first. Our chosen platform is EMQX.

If you have any questions, please feel free to communicate with us, you can contact zhenweiyu@whu.edu.cn.
