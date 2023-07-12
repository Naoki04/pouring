import numpy as np
import matplotlib.pyplot as plt

# set all position as a relative position from the starting point of the trajectory
def calc_relative(positions):
    relative_positions = positions.copy()
    initial = positions[0]
    for i in range(len(positions)):
        relative_positions[i][0] = positions[i][0] - initial[0]
        relative_positions[i][1] = positions[i][1] - initial[1]
    return relative_positions

xc,yc = 0,0
r = 75
n = 15

positions = np.ones((n,2))

for i in range(n):
    x = (r*np.cos(i/n * (2*np.pi))+xc )/2
    y = (r*np.sin(i/n * (2*np.pi))+yc )

    positions[i,0] = x
    positions[i,1] = y

positions = calc_relative(positions)




def main():
    print(positions)

    plt.plot(positions[:,0], positions[:,1])
    plt.show()

if __name__ == '__main__':
    main()
