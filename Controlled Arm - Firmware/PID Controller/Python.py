from __future__ import division, print_function
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg
np.set_printoptions(precision=4)
#from control import *

K_e = 0.0053 #from data
K_m =  K_e#motor coefficient
K_t = 1.78e-3 #from data
T = 0.001 #in seconds
J_m = 5e-6 #N*m
J_a = 4.5e-4  #N*m
R_m = 0.5 #in ohms
R_s = 0.5 #in ohms
L_a = 0.15 #in meters (15 cm)
K_d =0 #1e-2 drag on prop from arm movement...consider changing if you want!
K_f = 0 #friction coefficient consider changing if you want!


#three state plant model third state is Ke*omega
A = np.matrix([[]])
B = np.matrix([[]])
C = np.matrix([])
D = np.matrix([])
sys = ss(A,B,C,D,T)

#acker natural frequency picking
des_nat_freq = [0.9999,0.9999,0.9999]
k_acker = acker(A,B,des_nat_freq)

#print(np.linalg.matrix_rank(ctrb(A,B)))

print("A matrix:")
print(A)
print("B matrix:")
print(B)

k = k_acker
#k = np.matrix([[0,0,0]]) #use if you wish to manually set k values

#calculate K_u:
KuInv = C*np.linalg.inv(np.eye(3)-1*(A-B*k))*B
Ku = np.linalg.inv(KuInv)
print(Ku)
Ac = A-B*k #equivalent to M in notes
Bc = B*Ku
Cc = C
Dc = D

overall=ss(Ac,Bc,Cc,Dc,T)


#Simulate in Feedback:
t = np.arange(0,3,0.1)
sig = np.ones(np.size(t))
y,t1,x=lsim(overall,sig,t)


plot_in_radians = True
if plot_in_radians:
    scale = 1
else:
    scale = 180/3.14159

command = np.array(Ku*sig[1] - k*x)
plt.subplot(4, 1, 1)
x1, = plt.plot(scale*x[0], label='theta_a')
plt.ylabel('theta_a')
plt.subplot(4, 1, 2)
x2, = plt.plot(scale*x[1], label='omega_a')
plt.ylabel('omega_a')
plt.subplot(4, 1, 3)
x3, = plt.plot(x[2], label='K_e*omega_m')
plt.ylabel('back_emf')
plt.subplot(4, 1, 4)
com, = plt.plot(command[0], label='command')
plt.ylabel('command')
plt.xlabel('time steps')
plt.show()