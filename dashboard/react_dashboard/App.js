import React, { useState, useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './App.css';

// Fix for default marker icons in React-Leaflet
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon-2x.png',
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-shadow.png',
});

// Custom boat icon
const boatIcon = new L.Icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34]
});

// Custom waypoint icon
const waypointIcon = new L.Icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41]
});

// Component to update map view when boat moves
function MapUpdater({ position }) {
  const map = useMap();
  
  useEffect(() => {
    if (position) {
      map.setView(position, map.getZoom());
    }
  }, [position, map]);
  
  return null;
}

function App() {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [boatPosition, setBoatPosition] = useState([-33.8568, 151.2153]);
  const [boatHeading, setBoatHeading] = useState(0);
  const [waypoint, setWaypoint] = useState(null);
  const [telemetry, setTelemetry] = useState({
    gps: { lat: 0, lon: 0 },
    heading: 0,
    speed: 0,
    wind: 0,
    distance: 0
  });

  // Initialize ROS connection
  useEffect(() => {
    if (typeof window !== 'undefined' && window.ROSLIB) {
      const rosConnection = new window.ROSLIB.Ros({
        url: 'ws://localhost:9090'
      });

      rosConnection.on('connection', () => {
        console.log('Connected to ROS');
        setConnected(true);
      });

      rosConnection.on('error', (error) => {
        console.error('ROS Error:', error);
        setConnected(false);
      });

      rosConnection.on('close', () => {
        console.log('Disconnected from ROS');
        setConnected(false);
      });

      setRos(rosConnection);

      return () => {
        rosConnection.close();
      };
    } else {
      // Load ROSLIB from CDN if not available
      const script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/npm/roslib@1.3.0/build/roslib.min.js';
      script.onload = () => {
        const rosConnection = new window.ROSLIB.Ros({
          url: 'ws://localhost:9090'
        });
        setRos(rosConnection);
      };
      document.head.appendChild(script);
    }
  }, []);

  // Subscribe to GPS topic
  useEffect(() => {
    if (!ros || !connected) return;

    const gpsTopic = new window.ROSLIB.Topic({
      ros: ros,
      name: '/wamv/sensors/gps/gps/fix',
      messageType: 'sensor_msgs/NavSatFix'
    });

    gpsTopic.subscribe((message) => {
      if (message.latitude !== 0 && message.longitude !== 0) {
        const newPos = [message.latitude, message.longitude];
        setBoatPosition(newPos);
        setTelemetry(prev => ({
          ...prev,
          gps: { lat: message.latitude, lon: message.longitude }
        }));
      }
    });

    return () => {
      gpsTopic.unsubscribe();
    };
  }, [ros, connected]);

  // Subscribe to pose topic (for heading)
  useEffect(() => {
    if (!ros || !connected) return;

    const poseTopic = new window.ROSLIB.Topic({
      ros: ros,
      name: '/wamv/pose',
      messageType: 'geometry_msgs/Pose'
    });

    let lastPose = null;
    let lastTime = Date.now();

    poseTopic.subscribe((message) => {
      // Calculate heading from quaternion
      const q = message.orientation;
      const yaw = Math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      );
      const headingDeg = (yaw * 180 / Math.PI + 360) % 360;
      
      setBoatHeading(headingDeg);
      setTelemetry(prev => ({ ...prev, heading: headingDeg }));

      // Calculate speed
      const now = Date.now();
      const dt = (now - lastTime) / 1000.0;
      if (lastPose && dt > 0) {
        const dx = message.position.x - lastPose.position.x;
        const dy = message.position.y - lastPose.position.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        const speed = distance / dt;
        setTelemetry(prev => ({ ...prev, speed }));
      }
      lastPose = message;
      lastTime = now;
    });

    return () => {
      poseTopic.unsubscribe();
    };
  }, [ros, connected]);

  // Subscribe to wind topic
  useEffect(() => {
    if (!ros || !connected) return;

    const windTopic = new window.ROSLIB.Topic({
      ros: ros,
      name: '/vrx/debug/wind/direction',
      messageType: 'std_msgs/Float32'
    });

    windTopic.subscribe((message) => {
      const windDeg = (message.data * 180 / Math.PI + 360) % 360;
      setTelemetry(prev => ({ ...prev, wind: windDeg }));
    });

    return () => {
      windTopic.unsubscribe();
    };
  }, [ros, connected]);

  // Calculate heading arrow endpoint
  const getHeadingArrow = () => {
    if (!boatPosition) return null;
    
    const arrowLength = 0.0005; // ~50 meters
    const yaw = boatHeading * Math.PI / 180;
    const latOffset = arrowLength * Math.cos(yaw);
    const lonOffset = arrowLength * Math.sin(yaw) / Math.cos(boatPosition[0] * Math.PI / 180);
    
    return [
      boatPosition,
      [boatPosition[0] + latOffset, boatPosition[1] + lonOffset]
    ];
  };

  return (
    <div className="app">
      <div className="map-container">
        <MapContainer
          center={boatPosition}
          zoom={15}
          style={{ height: '100vh', width: '100%' }}
        >
          <TileLayer
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            attribution='© OpenStreetMap contributors'
          />
          
          <MapUpdater position={boatPosition} />
          
          {/* Boat marker */}
          <Marker position={boatPosition} icon={boatIcon}>
            <Popup>Boat Position</Popup>
          </Marker>
          
          {/* Heading arrow */}
          {getHeadingArrow() && (
            <Polyline
              positions={getHeadingArrow()}
              color="blue"
              weight={3}
              opacity={0.7}
            />
          )}
          
          {/* Waypoint marker */}
          {waypoint && (
            <>
              <Marker position={waypoint} icon={waypointIcon}>
                <Popup>Target Waypoint</Popup>
              </Marker>
              <Polyline
                positions={[boatPosition, waypoint]}
                color="red"
                weight={2}
                dashArray="5, 5"
                opacity={0.7}
              />
            </>
          )}
        </MapContainer>
      </div>
      
      <div className="sidebar">
        <h2>🚤 WAM-V Telemetry</h2>
        
        <div className={`status ${connected ? 'connected' : 'disconnected'}`}>
          {connected ? 'Connected' : 'Disconnected'}
        </div>
        
        <div className="telemetry-item">
          <div className="telemetry-label">Position</div>
          <div className="telemetry-value">
            {telemetry.gps.lat.toFixed(6)}, {telemetry.gps.lon.toFixed(6)}
          </div>
        </div>
        
        <div className="telemetry-item">
          <div className="telemetry-label">Heading</div>
          <div className="telemetry-value">{telemetry.heading.toFixed(1)}°</div>
        </div>
        
        <div className="telemetry-item">
          <div className="telemetry-label">Speed</div>
          <div className="telemetry-value">{telemetry.speed.toFixed(2)} m/s</div>
        </div>
        
        <div className="telemetry-item">
          <div className="telemetry-label">Wind Direction</div>
          <div className="telemetry-value">{telemetry.wind.toFixed(1)}°</div>
        </div>
        
        {waypoint && (
          <div className="telemetry-item">
            <div className="telemetry-label">Distance to Waypoint</div>
            <div className="telemetry-value">
              {telemetry.distance.toFixed(1)} m
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export default App;

