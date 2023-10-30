import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the obstacles' positions
obstacles = np.array([[1, 1], [1, -1], [-1, 1], [-1, -1]])

# Range parameter
range_val = 0.4

# Create a grid of points covering the space of interest
x = np.linspace(-range_val, range_val, 400)
y = np.linspace(-range_val, range_val, 400)
X, Y = np.meshgrid(x, y)

# Initialize subplots
fig = plt.figure(figsize=(12, 12))
axes = [fig.add_subplot(2, 2, i+1) for i in range(4)]

# Convert 2D axes to 3D for surface plots
for i in [1, 3]:
    axes[i] = fig.add_subplot(2, 2, i+1, projection='3d')

# Potential field functions
def potential_log(r):
    return -np.log(r + 1e-6)

def potential_inverse(r):
    return 1 / (r + 1e-6)

# Function to compute and plot the potential field
def plot_potential_field(ax, ax_3d, potential_func, label_text, cbar_range=None):
    Phi = np.zeros(X.shape)
    for obstacle in obstacles:
        r = np.sqrt((X - obstacle[0])**2 + (Y - obstacle[1])**2)
        Phi += potential_func(r)
    
    # Plot heatmap
    if cbar_range is not None:
        c = ax.pcolormesh(X, Y, Phi, shading='auto', cmap='jet', vmin=cbar_range[0], vmax=cbar_range[1])
    else:
        c = ax.pcolormesh(X, Y, Phi, shading='auto', cmap='jet')
    fig.colorbar(c, ax=ax, label='Potential Field Value ($\phi$)')
    ax.set_xlim(-range_val, range_val)
    ax.set_ylim(-range_val, range_val)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    # Adding text box as a legend
    ax.text(0.05, 0.95, label_text, transform=ax.transAxes, fontsize=12,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Plot 3D surface
    ax_3d.plot_surface(X, Y, Phi, cmap='jet', alpha=0.9, edgecolor='none')
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('$\phi$')

# Plot potential fields
plot_potential_field(axes[0], axes[1], potential_log, '$-\log(r)$')
plot_potential_field(axes[2], axes[3], potential_inverse, '$1/r$')

plt.tight_layout()
plt.show()
