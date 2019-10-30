import os
# Kill unwanted processess
[os.system("pkill -9 {}".format(proc)) for proc in ["px4","server","mavros_node","python","python2"]]

os.system("rosnode kill -a")
