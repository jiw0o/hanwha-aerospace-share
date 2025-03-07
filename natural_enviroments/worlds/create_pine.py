import xml.etree.ElementTree as ET
import random

# XML 텍스트
xml_text1 = """
<model name='pine_3_clone_clone_N'>
        <pose frame=''>X Y 4 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_0'>
          <pose frame=''>X Y 4 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='link_1'>
          <pose frame=''>X Y 4 1.57 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""

xml_text2 = """
<model name='pine_3_clone_clone_N'>
      <link name='link_0'>
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
        <pose frame=''>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>80</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/crown.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='link_1'>
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
        <pose frame=''>0 0 0 1.57 -0 0</pose>
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>90</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://pine_3/trunk.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose frame=''>X Y 4 0 -0 0</pose>
    </model>
"""


for n in range(21,120):
	root1 = ET.fromstring(xml_text1)
	root2 = ET.fromstring(xml_text2)
	root1.attrib['name'] = f'pine_3_clone_clone_{n}'
	root2.attrib['name'] = f'pine_3_clone_clone_{n}'
	random_X = format(random.uniform(-190, 500), '.3f')
	random_Y = format(random.uniform(-400, -135), '.3f')
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
	f1=open('/home/kim/catkin_ws/src/natural_environments_ros/natural_enviroments/worlds/pine1.txt','a')
	f1.write(new_xml_text1)
	f1.write("\n")
	f1.close()
	new_xml_text2 = ET.tostring(root2).decode()
	f2=open('/home/kim/catkin_ws/src/natural_environments_ros/natural_enviroments/worlds/pine2.txt','a')
	f2.write(new_xml_text2)
	f2.write("\n")
	f2.close()

print("complete!!!!!")

