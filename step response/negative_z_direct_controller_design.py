import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# ---------------------------
# Transfer Function
# ---------------------------
num = [-50.91, -508.2]
den = [1, 5.628, 11.38]
G = ctrl.TransferFunction(num, den)

# ---------------------------
# Step response (0 → -1)
# ---------------------------
t = np.linspace(0, 5, 1000)
t, y = ctrl.step_response(G, t)

# 將 unit step 改為 -1 step
y = y

# ---------------------------
# Plot
# ---------------------------
plt.figure(figsize=(7, 4))
plt.plot(t, y, linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.title('Step Response of G(s) (0 → -1)')
plt.grid(True)
plt.tight_layout()
plt.show()
