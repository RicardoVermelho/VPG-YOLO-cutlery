import numpy as np
def setup(option, idxobject):
    if option == 1:
        return scene1(idxobject)
    elif option == 2:
        return scene2(idxobject)
    '''elif option == 3:
    elif option == 4:
    elif option == 5:
    elif option == 6:
    elif option == 7:
    elif option == 8:
    elif option == 9:
    elif option == 10:'''

def scene1(idxobject):
    if idxobject == 0:
        return [-0.3, -0.1, 0.0001], [np.pi/4, (25/36)*np.pi, np.pi/2]
    elif idxobject == 1:
        return [-0.35, -0.1, 0.0001], [(25/36)*np.pi, np.pi/4, np.pi/2]
    else:
        return [-0.45, -0.2, 0.0001], [np.pi/2, 0, np.pi/2]

def scene2(idxobject):
    if idxobject == 0:
        return [-0.3, -0.1, 0.0001], [np.pi/4, (25/36)*np.pi, np.pi/2]
    elif idxobject == 1:
        return [-0.35, -0.1, 0.0001], [(25/36)*np.pi, np.pi/4, np.pi/2]
    elif idxobject == 2:
        return [-0.25, -0.2, 0.0001], [(17/18)*np.pi, (7/18)*np.pi, np.pi/2]
    elif idxobject == 3:
        return [-0.45, 0, 0.0001], [np.pi/2, 0, np.pi/2]
    elif idxobject == 4:
        return [-0.2, 0, 0.0001], [0, np.pi/2, np.pi/2]
    else:
        return [-0.45, -0.2, 0.0001], [np.pi/2, 0, np.pi/2]