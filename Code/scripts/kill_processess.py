import os
# Kill unwanted processess
[os.system("pkill -0 {}".format(proc)) for proc in ["px4","server","mavros_node","python","python2"]]
