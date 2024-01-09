### Injiget_char

<img src="https://github.com/melona0826/injiget_char/assets/61529916/7a597e1e-792c-439a-a03b-cddfef6bc43b" width="400" height="400"/>

## 💁 Introduction  
[![Video Label](http://img.youtube.com/vi/AUcnZOFWO9A/0.jpg)](https://youtu.be/AUcnZOFWO9A)   [![Video Label](http://img.youtube.com/vi/OJm6Rzef218/0.jpg)](https://youtu.be/OJm6Rzef218)  

**(⬆️ You can see the demostration video by click above images ⬆️)**

Injiget_char is OCR based Autonomous Forklift System Project.  
This project for the 2023 Spring KAIST CS470 (Introduction to Artificial Intelligence) [Prof. Daehyng Park]  
Our base platform is TURTLEBOT3 and we modify the hardware.  
The Block diagram of Injiget_char system is below.  

<img src="https://github.com/melona0826/injiget_char/assets/61529916/0cd7d0bc-577d-4abb-a0cd-77c397f6570c" width="500" height="300"/>

### Project Members

* [**Jin Kwon**](https://github.com/melona0826)
  >(KAIST, School of Computing)  
  >kwonjin@kaist.ac.kr

* **Myungwoo Jung**
  >(KAIST, Department of Mechanical Engineering)
* [**Dongwon Choi**](https://github.com/chlehdwon)
  >(KAIST, School of Computing)  
  >cookie000215@kaist.ac.kr 

* [**Youngwoo Jung**](https://github.com/beautifulpie)
  >(KAIST, Department of Chemistry)
  >jysys123@kaist.ac.kr
* **Taewoong Yun**
  >(KAIST, Department of Aerospace Engineering)


## 🌏 Environment

### 💻 System Environment
- **Ubuntu 20.04  + ROS Noetic**  

  (For install ubuntu 20.04 on Jetson Nano, we refer this page)  
  https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image


### 🪛 Hardware Environment
- **Jetson Nano**
- **TURTLEBOT3**
- **2 Servo Motors**
- **3D printed frame**


## 🛠️ Servo motor & Jetson Nano GPIO pin

We modify the hardware for build a Tilt & Fork system.  
So, before you launch this system, you must build hardware first ❗  
Below is the table of the GPIO pin of Jetson Nano connected each servo motor.  

|Servo Motor|GPIO Pin|
|:----------:|:----------:|
|Tilt Motor|  33  |
|Fork Motor|  32  |  

**⚠️ If you change the GPIO Pin of servo motors, you must change the pin number variable in the below files.**

|line number | file path|
|:----------:|:---------:|
|10 | servo_control/scripts/fork_control.py|
|10 | servo_control/scripts/tilt_control.py| 

## 💻 Install 

### ✔️ Injiget_char (Jetson Nano)
- ### usb_cam  
  We use usb_cam package usb_cam for streaming with usb webcam.
  
  **⚠️ You must type below command at your catkin workspace at Injiget_char (Jetson Nano)**
  ```
  git clone -b develop https://github.com/ros-drivers/usb_cam.git
  ```

- ### injiget_char  
  In our github, injiget_char branch is packages for injiget_char (Jetson Nano)
  
  **⚠️ You must type below command at your catkin workspace of Injiget_char (Jetson Nano)**
  ```
  git clone -b injiget_char https://github.com/melona0826/injiget_char.git
  ```

### ✔️ Host
- ### injiget_char  
  In our github, host branch is packages for injiget_char 
  
  **⚠️ You must type below command at your catkin workspace of host**
  ```
  git clone -b host https://github.com/melona0826/injiget_char.git
  ```

## 🏃 Launch

- ### ✔️ Injiget_char  (Jetson Nano)
  - Launch usb_cam launch file 
  ```
  roslaunch usb_cam usb_cam.launch
  ```
  
  - Launch injiget_char servo control launch file 
  ```
  roslaunch injiget_char servo_control.launch
  ```

- ### ✔️ Host  
  - Launch injiget_char start launch file 
  ```
  roslaunch injiget_char start.launch
  ```

##  👷‍♂️ Information & connect 

💁 If you want to know more detail information, you can refer our github WIKI page ❗

🤙 If you want to connect us, feel free to mail below mail address ❗

    kwonjin@kaist.ac.kr



