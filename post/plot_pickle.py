import pickle
import matplotlib.pyplot as plt

fs = open('/home/antond2/Desktop/Research/WiNerf-Data-Collection-Workspace/post/out/winerf_prelim2_post/trial_viz.pickle', 'rb')
p = pickle.load(fs)
p.show()
