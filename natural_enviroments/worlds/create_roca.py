import xml.etree.ElementTree as ET
import random

# XML 텍스트
xml_text1 = """
<model name='roca3mg_N'>
        <pose frame=''>372.361 -243.883 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_1'>
          <pose frame=''>X Y 100 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""

xml_text2 = """
<model name='roca3mg_N'>
      <link name='link_1'>
        <pose frame=''>-3e-06 4e-06 0 0 -0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://roca3/model.dae</uri>
              <scale>20 20 20</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>40</laser_retro>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://roca3/model.dae</uri>
              <scale>20 20 20</scale>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose frame=''>X Y 100 0 -0 0</pose>
    </model>
"""


for n in range(0,50):
	root1 = ET.fromstring(xml_text1)
	root2 = ET.fromstring(xml_text2)
	root1.attrib['name'] = f'roca3mg_{n}'
	root2.attrib['name'] = f'roca3mg_{n}'
	random_X = format(random.uniform(-500, 500), '.3f')
	random_Y = format(random.uniform(-500, 500), '.3f')
	for pose_element in root1.iter('pose'):
		original_values = pose_element.text.split()
		replaced_values_X = [random_X if value=='X' else value for value in original_values]
		replaced_values_XY = [random_Y if value=='Y' else value for value in replaced_values_X]
		pose_element.text = ' '.join(replaced_values_XY)
	for pose_element in root2.iter('pose'):
		original_values = pose_element.text.split()
		replaced_values_X = [random_X if value=='X' else value for value in original_values]
		replaced_values_XY = [random_Y if value=='Y' else value for value in replaced_values_X]
		pose_element.text = ' '.join(replaced_values_XY)
	new_xml_text1 = ET.tostring(root1).decode()
	f1=open('/home/kim/catkin_ws/src/natural_environments_ros/natural_enviroments/worlds/roca1.txt','a')
	f1.write(new_xml_text1)
	f1.write("\n")
	f1.close()
	new_xml_text2 = ET.tostring(root2).decode()
	f2=open('/home/kim/catkin_ws/src/natural_environments_ros/natural_enviroments/worlds/roca2.txt','a')
	f2.write(new_xml_text2)
	f2.write("\n")
	f2.close()

print("complete!!!!!")

