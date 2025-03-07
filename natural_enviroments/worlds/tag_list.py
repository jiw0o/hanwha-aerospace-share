model_class_dic = {
"terreno_lago/lago.dae":"earth",
"hill/hill_path.dae":"earth",
"terreno_parque/terreno_parque.dae":"earth",
"camino_parque/camino_parque.dae":"earth",
"dry_bush/untitled.obj":"bush",
"camino/materials/scripts":"camino",

"path/path.dae":"path",

"roca2/model.dae":"rock",
"roca3/model.dae":"rock",

"tree_8/crown8.obj":"foilage",
"tree_8/bark8.obj":"trunk",

"arbol1/copa.dae":"foilage",
"arbol1/tronco.dae":"trunk",

"arbol2/coparbol.dae":"foilage",
"arbol2/troncoarbol.dae":"trunk",

"arbol3/copa3.dae":"foilage",
"arbol3/tronco3.dae":"trunk",

"arbol3_dry/copa3.dae":"foilage",
"arbol3_dry/tronco3.dae":"trunk",

"pine_1/crown.obj":"foilage",
"pine_1/trunk.obj":"trunk",

"pine_2/crown.obj":"foilage",
"pine_2/trunk.obj":"trunk",

"pine_3/crown.obj":"foilage",
"pine_3/trunk.obj":"trunk",

"pine_5/crown.obj":"foilage",
"pine_5/trunk.obj":"trunk",

"pine_6/crown.obj":"foilage",
"pine_6/trunk.obj":"trunk",
}

class_tag_dic = {
"rock":(
"""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>1 0 0 1</emissive>
          </material>""",
"40"),
"foilage":(
"""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>1 1 0 1</emissive>
          </material>""",
"80"),
"trunk":(
"""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>1 0 1 1</emissive>
          </material>""",
"90"),
}

sky ="""
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>"""

sky_tagged ="\n"

terreno_parque ="""
          <material>
            <script>
              <uri>model://terreno_parque/materials/scripts</uri>
              <uri>model://terreno_parque/materials/textures</uri>
              <name>vrc/parque</name>
            </script>
          </material>"""

terreno_parque_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 1 0 1</emissive>
          </material>"""

terreno_parque_laser ="""
        <collision name='collision'>
          <laser_retro>1</laser_retro>
          <geometry>
            <mesh>
              <uri>model://terreno_parque/terreno_parque.dae</uri>"""

terreno_parque_laser_tagged ="""
        <collision name='collision'>
          <laser_retro>100</laser_retro>
          <geometry>
            <mesh>
              <uri>model://terreno_parque/terreno_parque.dae</uri>"""

hill ="""
          <material>
            <script>
              <uri>model://hill/materials/scripts</uri>
              <uri>model://hill/materials/textures</uri>
              <name>vrc/hill</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>1</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://hill/hill_path.dae</uri>"""

hill_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 1 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://hill/hill_path.dae</uri>"""

hill_path ="""
          <material>
            <script>
              <uri>model://path/materials/scripts</uri>
              <uri>model://path/materials/textures</uri>
              <name>vrc/path</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>1</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://path/path.dae</uri>"""

hill_path_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0.298 0.2235 0.1882 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://path/path.dae</uri>"""

terreno_lago ="""
          <material>
            <script>
              <uri>model://terreno_lago/materials/scripts</uri>
              <uri>model://terreno_lago/materials/textures</uri>
              <name>vrc/terreno_lago</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>1</laser_retro>"""

terreno_lago_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 1 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>"""

lago3 ="""
          <material>
            <script>
              <uri>model://lago3/materials/scripts</uri>
              <uri>model://lago3/materials/textures</uri>
              <name>vrc/agua</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>120</laser_retro>"""

lago3_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0.129 0.435 0.698</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>"""

lago_1 ="""
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <box>
              <size>75.0997 37.8189 8.6983</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://lago/materials/scripts</uri>
              <uri>model://lago/materials/textures</uri>
              <name>vrc/agua</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>"""

lago_1_tagged ="""
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <box>
              <size>75.0997 37.8189 8.6983</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0.129 0.435 0.698</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>10</max_contacts>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <box>
              <size>75.0997 37.8189 8.6983</size>
            </box>
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
        </collision>"""

lago ="""
          <geometry>
            <mesh>
              <uri>model://terreno_lago/lago.dae</uri>
              <scale>50 25 4</scale>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://terreno_lago/materials/scripts</uri>
              <uri>model://terreno_lago/materials/textures</uri>
              <name>vrc/terreno_lago</name>
            </script>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>1</laser_retro>"""

lago_tagged ="""
          <geometry>
            <mesh>
              <uri>model://terreno_lago/lago.dae</uri>
              <scale>50 25 4</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 1 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>"""

camino_parque ="""
          <material>
            <script>
              <uri>model://camino_parque/materials/scripts</uri>
              <uri>model://camino_parque/materials/textures</uri>
              <name>vrc/terrain</name>
            </script>
          </material>"""

camino_parque_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0.298 0.2235 0.1882 1</emissive>
          </material>"""

camino_parque_laser ="""
          <laser_retro>7</laser_retro>
          <geometry>
            <mesh>
              <uri>model://camino_parque/camino_parque.dae</uri>"""

camino_parque_laser_tagged ="""
          <laser_retro>50</laser_retro>
          <geometry>
            <mesh>
              <uri>model://camino_parque/camino_parque.dae</uri>"""

camino ="""
          <material>
            <script>
              <uri>model://camino/materials/scripts</uri>
              <uri>model://camino/materials/textures</uri>
              <name>vrc/mud</name>
            </script>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <emissive>0 0 0 1</emissive>
            <shader type='vertex'>
              <normal_map>__default__</normal_map>
            </shader>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>170</laser_retro>"""

camino_tagged ="""
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>1 0 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>170</laser_retro>"""

dry_bush ="""
          <geometry>
            <mesh>
              <uri>model://dry_bush/untitled.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <lighting>1</lighting>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.3961 0.2627 0.1994 1</ambient>
            <diffuse>0.3961 0.2627 0.1994 1</diffuse>
            <specular>0.01 0.1 0.01 1</specular>
            <emissive>0 0 0 1</emissive>
            <shader type='vertex'>
              <normal_map>__default__</normal_map>
            </shader>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>4</laser_retro>"""

dry_bush_tagged ="""
          <geometry>
            <mesh>
              <uri>model://dry_bush/untitled.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0 0 1 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>70</laser_retro>"""


tag_dic = {sky: sky_tagged,
          terreno_parque: terreno_parque_tagged,
          terreno_parque_laser: terreno_parque_laser_tagged,
          hill: hill_tagged,
          hill_path: hill_path_tagged,
          terreno_lago: terreno_lago_tagged,
          lago3: lago3_tagged,
          lago_1: lago_1_tagged,
          lago: lago_tagged,
          camino_parque: camino_parque_tagged,
          camino_parque_laser: camino_parque_laser_tagged,
          camino: camino_tagged,
          dry_bush: dry_bush_tagged,
          }