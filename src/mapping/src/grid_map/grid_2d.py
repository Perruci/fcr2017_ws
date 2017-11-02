from matplotlib import pyplot as plt
import matplotlib as mpl

class Grid2D:
    ''' Class designed to host and plot GridMap images '''

    def __init__(self):
        ''' Contructor to set default plot parameters '''
        self.cmap = mpl.colors.ListedColormap(['blue','black','red'])
        self.bounds=[-1.5,-0.5,0.5,1.5]
        self.norm = mpl.colors.BoundaryNorm(self.bounds, self.cmap.N)
        self.first_imshow = True


    def show_grid2d(self, grid, node_name):
        ''' Imshow a 2D grid based on grid np.matrix input: grid '''
        # tell imshow about color map so that only set colors are used
        self.img = plt.imshow(grid,interpolation='nearest',
                                 cmap = self.cmap, norm = self.norm)
        # first run of imshow
        if self.first_imshow:
            # make a color bar
            self.cbar = plt.colorbar(self.img,
                                     cmap=self.cmap,
                                     norm=self.norm,
                                     boundaries=self.bounds,
                                     ticks=[-1,0,1])
            self.cbar.ax.set_yticklabels(['free', 'unknown', 'obstacle'])
            self.first_imshow = False
        # iteractive mode: on
        plt.ion()
        # add plot title
        plt.title('Node: ' + node_name)
        # show plot
        plt.show()
        plt.pause(0.01)

    def save_grid2d(self, grid, node_name):
        if not self.first_imshow:
            plt.savefig(node_name + '.png', bbox_inches='tight')
