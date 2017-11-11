import matplotlib.pyplot as plt
import numpy as np

def bar_plot(pos, x=None, ylim=(0,1), title=None, c='#30a2da',
             **kwargs):
    """
    inspired by: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

    plot the values in `pos` as a bar plot.

    **Parameters**

    pos : list-like
        list of values to plot as bars

    x : list-like, optional
         If provided, specifies the x value for each value in pos. If not
         provided, the first pos element is plotted at x == 0, the second
         at 1, etc.

    ylim : (lower, upper), default = (0,1)
        specifies the lower and upper limits for the y-axis

    title : str, optional
        If specified, provides a title for the plot

    c : color, default='#30a2da'
        Color for the bars

    **kwargs : keywords, optional
        extra keyword arguments passed to ax.bar()

    """

    ax = plt.gca()
    ax.grid('on')
    if x is None:
        x = np.arange(len(pos))
    ax.bar(x, pos, color=c, **kwargs)
    if ylim:
        plt.ylim(ylim)
    plt.xticks(np.asarray(x)+0.4, x)
    if title is not None:
        plt.title(title)
