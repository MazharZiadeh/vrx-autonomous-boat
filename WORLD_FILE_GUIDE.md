# 🌍 World File (SDF) Modification Guide

## 📄 What is an SDF File?

**SDF** = **Simulation Description Format** (XML-based)

This is the file format used by Gazebo/VRX to define:
- The 3D world/environment
- All objects, models, and assets
- Physics properties
- Lighting, weather, water
- Camera settings
- Plugins and systems

## 📍 Your Current World File

**Location**: `/home/mazhar/final_stand/vrx/vrx_gz/worlds/sydney_regatta.sdf`

This is the world file currently being used when you launch:
```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

## 🎯 How to Modify Assets

### 1. **Add a New Model/Asset**

To add a new object to the world, add an `<include>` block:

```xml
<include>
  <name>my_custom_buoy</name>
  <pose>X Y Z ROLL PITCH YAW</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/model_name</uri>
</include>
```

**Example**: Add a red buoy at position (100, 200, 0):
```xml
<include>
  <name>my_red_buoy</name>
  <pose>100 200 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_red</uri>
</include>
```

### 2. **Remove an Asset**

Simply **delete** the entire `<include>...</include>` block for that asset.

**Example**: To remove the red buoy, delete:
```xml
<include>
  <name>mb_marker_buoy_red</name>
  <pose>-528 191 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_red</uri>
  <!-- ... rest of the block ... -->
</include>
```

### 3. **Modify Asset Position**

Change the `<pose>` values:
- **Format**: `<pose>X Y Z ROLL PITCH YAW</pose>`
- **X, Y, Z**: Position in meters
- **ROLL, PITCH, YAW**: Rotation in radians

**Example**: Move a buoy to a new location:
```xml
<include>
  <name>mb_marker_buoy_red</name>
  <pose>-400 150 0 0 0 0</pose>  <!-- Changed from -528 191 0 -->
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_red</uri>
</include>
```

### 4. **Change the Main Environment**

The main Sydney Regatta environment is loaded here (line 344-347):
```xml
<include>
  <pose>0 0 0.2 0 0 0 </pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/sydney_regatta</uri>
</include>
```

To use a different environment, change the `<uri>` to point to a different model.

### 5. **Modify Water/Waves**

The wave system is configured here (lines 603-636):
```xml
<plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
  <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters" every="2.0">
    params {
      key: "direction"
      value {
        type: DOUBLE
        double_value: 0.0  <!-- Wave direction in degrees -->
      }
    }
    params {
      key: "gain"
      value {
        type: DOUBLE
        double_value: 0.3  <!-- Wave height/amplitude -->
      }
    }
    params {
      key: "period"
      value {
        type: DOUBLE
        double_value: 5  <!-- Wave period in seconds -->
      }
    }
  </message>
</plugin>
```

### 6. **Modify Wind**

Wind is configured here (lines 581-601):
```xml
<plugin filename="libUSVWind.so" name="vrx::USVWind">
  <wind_direction>240</wind_direction>  <!-- Wind direction in degrees -->
  <wind_mean_velocity>0.0</wind_mean_velocity>  <!-- Wind speed in m/s -->
  <!-- ... -->
</plugin>
```

### 7. **Change Lighting**

The sun/light is defined here (lines 330-342):
```xml
<light type="directional" name="sun">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>  <!-- RGB color -->
  <direction>-0.5 0.1 -0.9</direction>  <!-- Light direction -->
</light>
```

### 8. **Change Sky/Background**

Sky and background are set here (lines 312-317):
```xml
<scene>
  <sky></sky>
  <grid>false</grid>
  <ambient>1.0 1.0 1.0</ambient>  <!-- Ambient light RGB -->
  <background>0.8 0.8 0.8</background>  <!-- Background color RGB -->
</scene>
```

## 📦 Available Models from Fuel

You can use models from Gazebo Fuel (online model repository):

**Common Models:**
- `mb_marker_buoy_red` - Red marker buoy
- `mb_marker_buoy_black` - Black marker buoy
- `mb_marker_buoy_green` - Green marker buoy
- `mb_marker_buoy_white` - White marker buoy
- `mb_round_buoy_orange` - Orange round buoy
- `mb_round_buoy_black` - Black round buoy
- `post` - Dock post
- `antenna` - Communication antenna
- `ground_station` - Ground station tent
- `blue_projectile` - Blue projectile ball

**Browse more models**: https://app.gazebosim.org/fuel/models

## 🔧 Structure of the World File

```
sydney_regatta.sdf
├── <world> (main container)
│   ├── <physics> - Physics engine settings
│   ├── <gui> - GUI/interface plugins
│   ├── <plugin> - System plugins (physics, sensors, etc.)
│   ├── <scene> - Scene settings (sky, background)
│   ├── <spherical_coordinates> - GPS coordinates
│   ├── <light> - Lighting
│   └── <include> - Models/assets (these are what you modify!)
│       ├── sydney_regatta (main environment)
│       ├── Coast Waves
│       ├── Buoys (red, black, green, white, orange)
│       ├── Posts
│       ├── Antenna
│       ├── Ground stations
│       └── Blue projectile
```

## 🎨 Example: Adding a Custom Obstacle Course

```xml
<!-- Add multiple buoys in a line -->
<include>
  <name>obstacle_buoy_1</name>
  <pose>-500 200 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_red</uri>
  <plugin name="vrx::PolyhedraBuoyancyDrag" filename="libPolyhedraBuoyancyDrag.so">
    <fluid_density>1000</fluid_density>
    <fluid_level>0.0</fluid_level>
    <linear_drag>25.0</linear_drag>
    <angular_drag>2.0</angular_drag>
    <buoyancy name="collision_outer">
      <link_name>link</link_name>
      <pose>0 0 -0.3 0 0 0</pose>
      <geometry>
        <cylinder>
          <radius>0.325</radius>
          <length>0.1</length>
        </cylinder>
      </geometry>
    </buoyancy>
    <wavefield>
      <topic>/vrx/wavefield/parameters</topic>
    </wavefield>
  </plugin>
</include>

<include>
  <name>obstacle_buoy_2</name>
  <pose>-450 200 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_green</uri>
  <!-- ... same plugin config ... -->
</include>
```

## 🚀 After Modifying

1. **Save the file**: `sydney_regatta.sdf`
2. **Restart the simulation**:
   ```bash
   # Stop current simulation
   pkill -f gazebo
   
   # Launch again
   cd ~/final_stand/vrx/dashboard
   ./LAUNCH_FULL_SYSTEM.sh
   ```

## 📝 Other World Files Available

You can also modify or create new world files:

- `navigation_task.sdf` - Navigation task world
- `stationkeeping_task.sdf` - Station keeping task
- `wayfinding_task.sdf` - Wayfinding task
- `gymkhana_task.sdf` - Gymkhana task
- `perception_task.sdf` - Perception task
- `wildlife_task.sdf` - Wildlife task
- `scan_dock_deliver_task.sdf` - Scan, dock, deliver task

**Location**: `/home/mazhar/final_stand/vrx/vrx_gz/worlds/`

## 🔍 Quick Reference

| Element | Purpose | Location in File |
|---------|---------|------------------|
| `<include>` | Add/remove models | Lines 344-579 |
| `<pose>` | Position & rotation | Inside `<include>` blocks |
| `<uri>` | Model source URL | Inside `<include>` blocks |
| `<plugin>` | Physics/behavior | Inside `<include>` blocks |
| `<light>` | Lighting | Lines 330-342 |
| `<scene>` | Sky/background | Lines 312-317 |
| `<plugin>` (wind) | Wind settings | Lines 581-601 |
| `<plugin>` (waves) | Wave settings | Lines 603-636 |

## ⚠️ Important Notes

1. **XML Syntax**: Make sure all tags are properly closed
2. **Indentation**: Keep consistent indentation for readability
3. **Coordinates**: 
   - X = East/West (positive = East)
   - Y = North/South (positive = North)
   - Z = Up/Down (positive = Up)
4. **Angles**: In radians (0 to 2π), or use degrees and convert
5. **Backup**: Always backup the original file before making changes!

## 🎯 Summary

**File Name**: `sydney_regatta.sdf`  
**Format**: SDF (Simulation Description Format) - XML  
**Location**: `/home/mazhar/final_stand/vrx/vrx_gz/worlds/`  
**Main Section to Edit**: `<include>` blocks (lines 344-579) for adding/removing assets

**To add assets**: Add new `<include>` blocks  
**To remove assets**: Delete `<include>` blocks  
**To modify assets**: Edit `<pose>` or `<uri>` inside `<include>` blocks

