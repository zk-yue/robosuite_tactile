# 1 触觉传感器添加

重点注意以下几点：

1. mesh的名字及分组：参与碰撞的物体后缀增加“\_collision”，设置group="0"；可以看到的物体后缀增加“\_visual”，group="1"。
2. 不参与碰撞的物体，设置contype="0" conaffinity="0"。

```xml
<!-- tactile -->
<body name="digit_0" pos="-0.0025 0.01 0.08" euler="1.5708 0 -1.5708"> 
<!-- 左右 前后 上下 -->
    <!-- <include file="digit0_sensor.xml"></include> -->
    <body name="digit0_sensor" pos="0 0 0" euler="0 0 0">
        <site name="digit0_site" pos="0.012 0 0.035" rgba="1 0 0 .0" size="0.02 0.02 0.02" group="1"/>
        <!-- <joint name="digit_joint" type="hinge" damping="0.01"></joint> -->
        <!--Front and Back-->
        <geom type="mesh" group="0" name="digit0_back_collision" euler="3.1416 0 1.5708" pos="0.028 0.015 0.036" material="black_resin" mesh="digit_back"  mass="0.05" friction="1 0.05 0.01"/>
        <geom type="mesh" contype="0" conaffinity="0" group="1" name="digit0_back_visual" material="black_resin"  mesh="digit_back" mass="0.0005" euler="3.1416 0 1.5708" pos="0.028 0.015 0.036"/>
        <!-- <geom name="digit0_adapter"  type="mesh" euler="0 -1.5708 3.1416" pos="0.05 0.0097 0.0326" material="silver" mesh="digit_adapter"  mass="0.05" contype="32" conaffinity="32" friction="1 0.05 0.01" solimp="1.1 1.2 0.001 0.5 2" solref="0.02 1"/> -->
        <!--Glass Cover-->
        <geom type="mesh" contype="0" conaffinity="0" group="1" name="digit0_glass_visual" material="transparent_glass"  mesh="digit_glass" mass="0.005"   pos="0.024 -0.0085 0.017" euler="0 0 1.5708"/>
        <!--Elastomer-->
        <geom type="mesh" contype="0" conaffinity="0" group="1" name="digit0_elastomer_visual" mesh="digit_gel" pos="0.024 -0.0085 0.032" euler="0 0 1.5708" rgba="0.9 0.95 1 0.0"/>
        <!--Elastomer Cover-->
        <!-- <geom name="digit0_elastCover" type="mesh" mesh="digit_gel_cover" pos="0.025 -0.01 0.031" euler="0 0 1.5708" contype="0" conaffinity="0" material="silver"
                friction="1 0.05 0.01" solimp="1.1 1.2 0.001 0.5 2" solref="0.02 1"/> -->
        <geom type="mesh" contype="0" conaffinity="0" group="1" name="digit0_elastCover_visual" mesh="digit_curve" pos="0.024 -0.0083 0.0305" euler="0 0 1.5708" material="silver"
            friction="1 0.05 0.01" solimp="1.1 1.2 0.001 0.5 2" solref="0.02 1"/>
        <!--Gel Camera-->
        <camera name="digit0_camera" mode="fixed" pos="0.0129 0.0014 0.008" euler="0 3.14159 1.5708" fovy="30"/>
        <!-- <site name="digit0_pos" type="sphere" pos="0.0129 0.0014 0.008" rgba="1 0 1 1" size=".008 .008 .008"/> -->
        <!-- Friction placeholder -->
        <geom name="digit0_friction_collision" group="0" type="box" size="0.013 0.013 0.00001" euler="0 0 0" pos="0.012 0.001 0.0372" rgba="0 0 1 0"
                friction="1 0.05 0.01"/>
    </body>
</body>
<!-- tactile -->
```