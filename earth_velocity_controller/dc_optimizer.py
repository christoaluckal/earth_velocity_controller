import numpy as np
from scipy.interpolate import CubicSpline, interp1d


def bucket_cmd(x):
    if x < -0.9: return -0.7464773197463939
    elif x < -0.8: return -0.7556077862660108
    elif x < -0.7: return -0.7475886335802124
    elif x < -0.6: return -0.6943159340316355
    elif x < -0.5: return -0.3478137666597351
    elif x < -0.45: return -0.11074819014084174
    elif x >= -0.45 and x <= 0.45: return x*0.005
    elif x < 0.45: return -0.03170411937007261
    elif x < 0.5: return 0.07074540207098638
    elif x < 0.6: return 0.1531096691607231
    elif x < 0.7: return 0.494843938147395
    elif x < 0.8: return 0.9654822056154688
    elif x < 0.9: return 1.1323096288265486
    elif x < 1.0: return 1.124019310302267
    else: return 1.1221103605940879

signal = np.arange(-1.0, 1.0, 0.1)
power =  np.array([bucket_cmd(x) for x in signal])
cs = CubicSpline(signal, power)

print(cs(-0.47))

import matplotlib.pyplot as plt
plt.plot(signal, power, 'o', label='data points')
plt.plot(signal, cs(signal), label='Cubic Spline')
plt.legend()
plt.show()

from scipy.optimize import minimize

def objective(x,w):
    error = (w-cs(x[0]))**2
    return error

def optimize_cmd(w):
    x_guesses = np.linspace(-1.0, 1.0, 20)
    best_x = None
    best_error = float('inf')
    for x0 in x_guesses:
        result = minimize(objective, [x0], args=(w,), bounds=[(-1.0, 1.0)])
        if result.fun < best_error:
            best_error = result.fun
            best_x = result.x[0]
    return best_x

if __name__ == "__main__":
    while True:
        w = float(input("Enter bucket velocity (-1.0 to 1.0): "))
        if w < -1.0 or w > 1.0:
            print("Invalid")
            continue
        x_opt = optimize_cmd(w)
        print(f"Optimal command for velocity {w} is: {x_opt}")


# import numpy as np
# from scipy.interpolate import CubicSpline

# np.set_printoptions(precision=4, suppress=True)

# def bucket_cmd(x):
# 	if abs(x) <= 0.45: return 0.0
# 	if x < -0.9: return -0.7464773197463939
# 	elif x < -0.8: return -0.7556077862660108
# 	elif x < -0.7: return -0.7475886335802124
# 	elif x < -0.6: return -0.6943159340316355
# 	elif x < -0.5: return -0.3478137666597351
# 	elif x < -0.45: return -0.11074819014084174
# 	elif x < 0.45: return -0.03170411937007261
# 	elif x < 0.5: return 0.07074540207098638
# 	elif x < 0.6: return 0.1531096691607231
# 	elif x < 0.7: return 0.494843938147395
# 	elif x < 0.8: return 0.9654822056154688
# 	elif x < 0.9: return 1.1323096288265486
# 	elif x < 1.0: return 1.124019310302267
# 	else: return 1.1221103605940879

# signal = np.arange(-1.0, 1.0, 0.1)
# power =  np.array([bucket_cmd(x) for x in signal])
# cum = []

# for i in range(len(power)):
# 	for j in range(len(power)):
# 		if i == j: continue
# 		else:
# 			cum.append([signal[i],signal[j], power[i]-power[j]])

# cum = np.array(cum)

# sorted = cum[np.argsort(cum[:,2])]
# print(sorted)