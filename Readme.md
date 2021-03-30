# Computational Control System Lab

The repository contains my __assignment solutions__ and __term project__ of the [Control Systems](https://www.bits-pilani.ac.in/hyderabad/EEE/ControlSystemsLab) course and [Computational Control System Lab](https://www.bits-pilani.ac.in/hyderabad/EEE/ControlSystemsLab) course and BITS Pilani. 

## The term project details are as follows 

The Term project deals with developement of an **propeller controlled arm design using a microcontroller**. 
The final model developed can be seen in the following image :

![Final Picture](https://courses.edx.org/assets/courseware/v1/6cdb6fb89f30026f9e3157be354c34a4/asset-v1:MITx+6.302.1x+2T2016+type@asset+block/on_scale.jpg)


### The hardware schematic for the same is shown below : 

![Schematic](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/schematic.png)

Here, we are using [Teensy 3.2](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcS_RRz-ZLV89quRsmHHOWXmTu9NQJqWvVU_gQ&usqp=CAU) along with Arduino Uno board for controlling the propellers. 


### The Soltware component is divided into following 3 parts 

  *Microcontroller Code: This is the code uploaded to the Arduino (the .ino file). It sets the Arduino up for listening to and controlling the electronics as well as taking in parameters and reporting values to your computer over serial (the USB cable). It is written in Arduino's C/C++ type language.

  *Server Code: This is a Python script that talks with the Arduino over serial, does some calculations, and then talks with the GUI code in your browser. It is the "middle-person" of our software stack.

  *GUI Code: Your GUI (Graphical User Interface) is basically just a web-page that is hosted on your machine. It is written in standard html/css/javascript. Don't worry, it is locally hosted and only you have access to it! It lives at localhost:3000 in your browser url field when the server script is running.

The controller GUI would look like this 
![GUI](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/gui.png) 

I have also attempted to build a flask server for developing a GUI version for the outputs of Arduino (for testing purpose). There are many bugs with the GUI currently, and I am open to any pull requests. The requirements for the project are present in req.txt file. 

### The Control Model of the System
The controller of the system is designed using a simple PD controller with an steady state offset. 

```C++
    // calculating the motor command signal here:
  req = kp*errorV + kd*andleVderv + directV;
  float motorCmd = req;

```

Individual folders contains readme files for improved explianations. Also the file **[parameter_derivation.md](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Parameter_derivation.md)** contains the model derivation. 

![Final Results](https://courses.edx.org/assets/courseware/v1/5c9b579190597487510b5838ccf817a3/asset-v1:MITx+6.302.1x+2T2016+type@asset+block/example_set_of_nat_freq.png)


---

## The projects are briefed below. 

  1. **Control of Motors** - Fuzzy PID controller design for a closed loop system of a DC shunt motor. The block diagram developed looks like this 

### DC Motor Block Diagram
![Fuzzy PID](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Capture.JPG)
  
  2. **Root Locus and Nyquist Plot Development for Stability analysis** - This assignment involved choosing of appropriate Nyquist contour for an open loop system and determine the values of static gain for which the system's closed loop reains stable. Later the same is done using plotting the root locus of the system. The hand written solution with the matlab code for the same is available [here - 2018A8PS0507P.pdf](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/2018A8PS0507P.pdf).

### Developed Plots
  ![Nyquist Plot](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Nyquist%20plot.PNG) 
  ![Root Locus](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Root%20locus.PNG)

  3. **Development of customized PD Controller** - Designing an Cascaded PD controller, with the **desired settling time and peak overshoot** is a very important task in any design algorithm. The hand written solution is present [here](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/2018A8PS0507P.pdf).

  4.  **Using Root Locus to control a 2nd Order closed loop P-only controller** - This involved the use of above 2 techniques to develop a customized controller. You can see the code present in the [folder](https://github.com/Jash-2000/Computational-Control-System-Lab/tree/master/Tuning%20of%20P%20controller%20of%202nd%20Order%20system%20using%20Root%20Locus) .The result obtained is as follows 

  ### Root Locus based P-only controller
![Step Response](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Step_Response.JPG)

  5. **Inveted Pendulum Control using intelligent Control Schemes** - This project contains the tuned versions of inverted pendulum using Fuzzy Genetic PID controllers. I have dedicated an entire repository on this topic, the link for which is present here : [Pole-Balance-Control-Algorithms](https://github.com/Jash-2000/Pole-Balance-Control-Algorithms). 
  **Please do star the repository if it helped you.**

  6. **Bode Plotting and Block Reduction Technique** - Experimeted with various functions of MATLAB to develop bode plots and examine the system dynamics. A complex block diagram was also reduced to a simple transfer function. 

  ### Complex Block Diagram 

  ![BD](https://github.com/Jash-2000/Computational-Control-System-Lab/blob/master/Block%20Diagram.JPG)

  The resolved transfer function was 

  ```MATLAB
    c = [1 , 3];
    r = [1 , 9 , 131 , 850 , 2142 , 1820];

    sys = tf(c,r);
  ```

---

## Resources

**__P.S__ - The write Ups and the codes for some of these group assignments are present with [Akshit Patel](https://github.com/Akshit-Patel/Control-System-Projects).**

  * [https://www.youtube.com/watch?v=iOmqgewj5XI&feature=emb_title](https://www.youtube.com/watch?v=iOmqgewj5XI&feature=emb_title)
  * [Understanding Basics of Fuzzy System](https://www.youtube.com/watch?v=OVINlUaEiS8&list=PLk8_UfafEClpL0r1YaYHz-F1AbGJDQizL)
  * [Root Locus](https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlRootLocus)
  * [Inverted Pendulum](http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=SystemModeling)
  * [Term Project](https://learning.edx.org/course/course-v1:MITx+6.302.1x+2T2016/home)