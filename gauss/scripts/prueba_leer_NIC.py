import pandas as pd
import numpy as np
df = pd.read_csv('/home/josmilrom/catkin_ws/src/jamrepo/Data_Storage/U&Aparameters/NIC_v1.csv', sep=',')
print df
b = df[(df.TC == 5) & (df.NICs == 0)]
print b[['Rc','VPL']]
print b['VPL'][0]

