# 📊 Telemetry Panel Guide

## ✅ What Was Implemented

Complete real-time telemetry panel with 5 sections:

### 1. Connection Status
- **ROS**: Shows "Connected ✓" or "Disconnected ✗"
- **Topics**: Shows "subscribed / available" count
- Color-coded: Green (connected), Red (disconnected)

### 2. Position Data
- **Latitude**: 6 decimal places
- **Longitude**: 6 decimal places  
- **Altitude**: Meters (from GPS altitude)
- Updates: ~10 Hz (on every GPS message)

### 3. Motion Data
- **Speed**: Knots (calculated from pose changes)
- **Heading**: Degrees (0-360°, 3-digit display)
- **Turn Rate**: Degrees/second (calculated from heading changes)
- Updates: Real-time as data arrives

### 4. Environment
- **Wind Direction**: Cardinal direction + degrees (e.g., "NW (315°)")
- **Wind Speed**: Knots (converted from m/s)
- **Wave Height**: Meters (shows "N/A" if not available)
- Updates: When wind topics publish

### 5. Mission Status
- **Current WP**: Current waypoint number / total
- **Distance**: Meters to current waypoint
- **ETA**: Estimated time of arrival (MM:SS format)
- **Status**: Mission status text ("En route", "Waiting...", etc.)
- **Progress Bar**: Visual progress indicator
- Updates: When waypoint data available

## 🎨 Styling Features

- **Dark Background**: #1a1a1a
- **Monospace Font**: Courier New for all numbers
- **Color Coding**:
  - Green (#4CAF50): Good values, connected status
  - Yellow (#FFC107): Warnings
  - Red (#f44336): Errors, disconnected
- **Update Animation**: Values flash green when updated
- **Smooth Transitions**: 0.3s ease for all updates

## 📡 Update Functions

### Position Telemetry
```javascript
function updatePositionTelemetry() {
    const { lat, lon, alt } = boatState.position;
    updateValue('lat-value', lat, 'latlon');
    updateValue('lon-value', lon, 'latlon');
    updateValue('altitude-value', alt, 'float');
}
```

### Motion Telemetry
```javascript
function updateMotionTelemetry() {
    updateValue('speed-value', boatState.speed, 'float');
    updateValue('heading-value', boatState.heading, 'heading');
    updateValue('turn-rate-value', boatState.turnRate, 'float');
}
```

### Environment Telemetry
```javascript
function updateEnvironmentTelemetry() {
    // Converts wind direction to cardinal (N, NE, E, etc.)
    // Updates wind speed (converted to knots)
    // Shows wave height or "N/A"
}
```

### Mission Status
```javascript
function updateMissionTelemetry() {
    // Updates waypoint progress
    // Calculates ETA from speed and distance
    // Updates status text with color coding
}
```

## 🔄 Update Triggers

| Data | Trigger | Frequency |
|------|---------|-----------|
| Position | GPS message | ~10 Hz |
| Heading | IMU message | ~10 Hz |
| Speed | Pose message | ~10 Hz |
| Turn Rate | IMU message | ~10 Hz |
| Wind | Wind topics | When available |
| Mission | Waypoint topic | When available |

## 🧪 Test It

1. **Open dashboard**: http://localhost:8000/dashboard.html
2. **Check initial state**: All values show "--" or defaults
3. **Connect ROS**: Status changes to "Connected ✓"
4. **Watch updates**: Values update with green flash animation
5. **Verify calculations**: Turn rate, ETA, etc. should be accurate

## 📝 HTML Structure

Each section follows this pattern:
```html
<div class="telemetry-section">
    <div class="section-title">Section Name</div>
    <div class="telemetry-panel">
        <div class="telemetry-row">
            <span class="telemetry-label-small">Label:</span>
            <span id="value-id" class="telemetry-value-small">--</span>
            <span class="telemetry-unit-small">unit</span>
        </div>
    </div>
</div>
```

## ✅ Ready!

The telemetry panel is fully functional and updates in real-time as ROS data arrives!

