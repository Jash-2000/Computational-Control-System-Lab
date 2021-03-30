# Computational Control System Lab

The repository contains my __assignment solutions__ and __term project__ of the Control Systems course and Computational Control System Lab course and BITS Pilani. 

## The term project details are as follows 




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

  ## P.S - The write Ups and the codes for some of these group assignments are present with [Akshit Patel](https://github.com/Akshit-Patel/Control-System-Projects).