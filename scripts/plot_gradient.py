import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the coordinates of the obstacles
obstacles = [(-1, 1), (-1, -1), (1, 1), (1, -1)]

# Create a grid of points for the environment with higher resolution
x = np.linspace(-2, 2, 800)
y = np.linspace(-2, 2, 800)
X, Y = np.meshgrid(x, y)

# Define the potential field functions and their gradients
def potential_and_gradient(X, Y, obstacles, function_type):
    phi_total = np.zeros_like(X)
    grad_phi_x_total = np.zeros_like(X)
    grad_phi_y_total = np.zeros_like(Y)
    
    for obs in obstacles:
        obs_x, obs_y = obs
        dx, dy = X - obs_x, Y - obs_y
        R = np.sqrt(dx**2 + dy**2)
        
        if function_type == 'log':
            phi = np.log2(R, where=R!=0, out=np.full_like(R, -np.inf))
            dphi_dx = dx / (R**2 * np.log(2))
            dphi_dy = dy / (R**2 * np.log(2))
        elif function_type == 'inverse_negative':
            phi = -1 / R
            dphi_dx = dx / R**3
            dphi_dy = dy / R**3
        elif function_type == 'gaussian':
            phi = -np.exp(-9 * R**2)
            dphi_dx = 18 * dx * np.exp(-9 * R**2)
            dphi_dy = 18 * dy * np.exp(-9 * R**2)
        else:
            raise ValueError("Invalid function type")
            
        phi_total += phi
        grad_phi_x_total += dphi_dx
        grad_phi_y_total += dphi_dy
    
    grad_phi_total = np.sqrt(grad_phi_x_total**2 + grad_phi_y_total**2)
    return phi_total, -grad_phi_total  # Invert the gradient magnitude here

# Plot the obstacles and gradient magnitudes of the potential fields
fig = plt.figure(figsize=(12, 10))
ax1 = fig.add_subplot(2, 2, 1)
ax1.scatter(*zip(*obstacles), s=100, color='red', zorder=5)
ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.grid(True)
ax1.set_title('Obstacles')

function_types = ['log', 'inverse_negative', 'gaussian']
titles = ['-|∇log2(r)|', '-|∇(-1/r)|', '-|∇(-exp(-9r^2))|']
zlims = [(-40, 0), (-800, 0), None]  # Define z-axis limits for each plot

for i, function_type in enumerate(function_types):
    ax = fig.add_subplot(2, 2, i + 2, projection='3d')
    phi, grad_phi = potential_and_gradient(X, Y, obstacles, function_type)
    surf = ax.plot_surface(X, Y, grad_phi, cmap='jet', alpha=0.6, linewidth=0, antialiased=True)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_xlabel('x/m')
    ax.set_ylabel('y/m')
    ax.set_zlabel('$\\phi$')
    ax.set_title(titles[i])
    if zlims[i] is not None:
        ax.set_zlim(*zlims[i])
    ax.grid(True)
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)

# Adjust spacing
plt.subplots_adjust(wspace=0.1, hspace=0.1)

plt.show()