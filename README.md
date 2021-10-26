# ros_stm32_arduino

Descargar Arduino 1.85, decomprimir e instalar

```
cd Downloads/arduino-1.8.5
./install.sh
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
```

Instalar los paquetes de ros para aduino

```
sudo apt-get install ros-melodic-rosserial
sudo apt-get install ros-melodic-rosserial-arduino
```


```
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```


```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
```



