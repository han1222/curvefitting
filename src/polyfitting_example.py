#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('fig3_1_tiltingRL.dat')

x = data[:,0]
y= data[:,1]
#plt.xscale('log')#plt.set_xscale('log')
coefs = np.polyfit(x, y, 10)
ffit = np.polyval(coefs, x)

new_x = np.linspace(0,26)
new_ffit = np.polyval(coefs, new_x)

plt.plot(x, y, 'o', label="Raw")
plt.plot(x, ffit,'x',label="Fit to Raw")
plt.plot(new_x, new_ffit,label="Fit to LinSpace")

# This is ugly. I'd use list comprehension here!
arr = np.linspace(0,26,20)
new_y = []
for xi in arr:
    total = 0
    for i,v in enumerate(coefs[::-1]):
        total += v*xi**i
    new_y.append(total)


plt.plot(arr, new_y, '*', label="Polynomial")

plt.legend(loc=2)
plt.show()