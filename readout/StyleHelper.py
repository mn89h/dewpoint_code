# -*- coding: utf-8 -*-
"""
Created on Fri Mar  1 10:19:06 2024

@author: malte
"""
import matplotlib.pyplot as plt

TEXTWIDTH_MASTER = 418.25555
IES_BLUE = '004c8b'
IES_RED = 'b81068'
IES_YELLOW = 'f5a301'
IES_BLUE_100 = '#004c8bff'
IES_RED_100 = '#b81068ff'
IES_YELLOW_100 = '#f5a301ff'

MASTER_STYLE_NORMAL = {
    "font.family": "sans-serif",
    # use inline math for ticks
    "text.usetex": True,
    "text.latex.preamble" : "\\usepackage{siunitx}",
    "pgf.rcfonts": False,
    # Use 10pt font in plots, to match 10pt font in document
    "axes.labelsize": 11,
    "font.size": 11,
    # Make the legend/label fonts a little smaller
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11
    }
MASTER_STYLE_SMALL = {
    "font.family": "sans-serif",
    # use inline math for ticks
    "text.usetex": True,
    "pgf.rcfonts": False,
    # Use 10pt font in plots, to match 10pt font in document
    "axes.labelsize": 11,
    "font.size": 11,
    # Make the legend/label fonts a little smaller
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9
    }


@staticmethod
def color(color, opacity_percent):
    byte = round(2.55 * opacity_percent)
    opacity_str = f'{byte:02x}' # convert to hex string
    return '#' + color + opacity_str
    

@staticmethod
def update_pyplot_rcParams(style=MASTER_STYLE_NORMAL):
    plt.style.use('default')
    plt.rcParams.update(style)


@staticmethod
def set_size(width_pt=TEXTWIDTH_MASTER, fraction=1, subplots=(1, 1)):
    """Set figure dimensions to sit nicely in our document.

    Parameters
    ----------
    width_pt: float
            Document width in points
    fraction: float, optional
            Fraction of the width which you wish the figure to occupy
    subplots: array-like, optional
            The number of rows and columns of subplots.
    Returns
    -------
    fig_dim: tuple
            Dimensions of figure in inches
    """
    # Width of figure (in pts)
    fig_width_pt = width_pt * fraction
    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    golden_ratio = (5**.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])

    return (fig_width_in, fig_height_in)
    
@staticmethod
def mscatter(x,y,ax=None, m=None, **kw):
    import matplotlib.markers as mmarkers
    if not ax: ax=plt.gca()
    sc = ax.scatter(x,y,**kw)
    if (m is not None) and (len(m)==len(x)):
        paths = []
        for marker in m:
            if isinstance(marker, mmarkers.MarkerStyle):
                marker_obj = marker
            else:
                marker_obj = mmarkers.MarkerStyle(marker)
            path = marker_obj.get_path().transformed(
                        marker_obj.get_transform())
            paths.append(path)
        sc.set_paths(paths)
    return sc
