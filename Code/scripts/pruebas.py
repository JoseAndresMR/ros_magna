import os
import xml.etree.ElementTree

et = xml.etree.ElementTree.parse('/home/joseandresmr/catkin_ws/src/pydag/Code/launch/complete_spawner_JA.launch')

print et

root = et.getroot()
self.ID = 2
root[7].attrib["ns"] = '$(arg ns_prefix){}'.format(self.ID)
root[7][0].attrib["value"] = str(self.ID)
root[8].attrib["ns"] = '$(arg ns_prefix){}'.format(self.ID)
root[8][0].attrib["value"] = str(self.ID)
root[9].attrib["name"] = 'uav_{}'.format(self.ID)
root[9].attrib["args"] = '-self.ID={}'.format(self.ID)
root[11][0].attrib["value"] = str(self.ID)

et.write('/home/joseandresmr/catkin_ws/src/pydag/Code/launch/complete_spawner_prueba_JA.launch')