import numpy as np
import matplotlib.pyplot as plt

eps = np.linspace(0.001,1.,100)*np.pi/180.
dGamma = eps*0.5
gamma0 = 35.*np.pi/180.

N = np.maximum(np.zeros(100), np.floor(np.log2((1./np.cos(gamma0)-1.)/(1./np.cos(dGamma)-1.))))

plt.figure()
plt.plot(eps,N)
plt.show()
