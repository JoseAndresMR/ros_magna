import pandas as pd
df = pd.read_csv('/home/josmilrom/catkin_ws/src/jamrepo/Data_Storage/U&Aparameters/NIC.csv', sep=',')
a = df["Rc"][df[(df.TC == 7) & (df.NICs == 0)].index]
print a