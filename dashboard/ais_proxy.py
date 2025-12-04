#!/usr/bin/env python3
"""
AIS Proxy - Connects to AISStream.io and forwards data to:
1. ROS topics (for Gazebo visualization)
2. WebSocket (for dashboard visualization)
"""

import asyncio
import json
import os
import sys
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

# AISStream API Key
AISSTREAM_API_KEY = os.getenv('AISSTREAM_API_KEY', '70e4762d6c570a3204ec48ddac3c08e1649320a5')

# WebSocket clients for dashboard
dashboard_clients = set()

# Store AIS vessels
ais_vessels = {}  # MMSI -> vessel data

async def handle_dashboard_client(websocket):
    """Handle dashboard WebSocket connections"""
    dashboard_clients.add(websocket)
    print(f"✓ Dashboard client connected (total: {len(dashboard_clients)})")
    
    # Send current vessels to new client
    if ais_vessels:
        for mmsi, vessel in ais_vessels.items():
            message = {
                "type": "vessel_update",
                "mmsi": mmsi,
                "data": vessel
            }
            try:
                await websocket.send(json.dumps(message))
            except:
                pass
    
    try:
        await websocket.wait_closed()
    finally:
        dashboard_clients.remove(websocket)
        print(f"✗ Dashboard client disconnected (remaining: {len(dashboard_clients)})")

async def broadcast_to_dashboard(message):
    """Broadcast message to all dashboard clients"""
    if dashboard_clients:
        disconnected = set()
        for client in dashboard_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)
        
        # Remove disconnected clients
        dashboard_clients.difference_update(disconnected)

async def connect_to_aisstream():
    """Connect to AISStream.io and process messages"""
    uri = f"wss://stream.aisstream.io/v0/stream?apiKey={AISSTREAM_API_KEY}"
    
    print(f"🔌 Connecting to AISStream.io...")
    print(f"   API Key: {AISSTREAM_API_KEY[:10]}...")
    
    while True:
        try:
            async with connect(uri) as websocket:
                print("✅ Connected to AISStream.io")
                
                # Subscribe to AIS messages (Sydney Regatta area)
                subscribe_message = {
                    "APIKey": AISSTREAM_API_KEY,
                    "BoundingBoxes": [[
                        [-34.0, 150.0],  # Southwest corner (Sydney area)
                        [-33.0, 151.5]   # Northeast corner
                    ]]
                }
                
                await websocket.send(json.dumps(subscribe_message))
                print("✅ Subscribed to AIS messages (Sydney area)")
                
                # Process incoming messages
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await process_ais_message(data)
                    except json.JSONDecodeError as e:
                        print(f"⚠️ JSON decode error: {e}")
                    except Exception as e:
                        print(f"⚠️ Error processing message: {e}")
                        
        except Exception as e:
            print(f"❌ Connection error: {e}")
            print("   Retrying in 5 seconds...")
            await asyncio.sleep(5)

async def process_ais_message(data):
    """Process AIS message and forward to ROS and dashboard"""
    try:
        if data.get("MessageType") == "PositionReport":
            metadata = data.get("MetaData", {})
            message = data.get("Message", {}).get("PositionReport", {})
            
            mmsi = metadata.get("MMSI")
            if not mmsi:
                return
            
            lat = message.get("Latitude")
            lon = message.get("Longitude")
            heading = message.get("TrueHeading", 0)
            speed = message.get("Sog", 0)  # Speed over ground (knots)
            course = message.get("Cog", 0)  # Course over ground
            ship_name = metadata.get("ShipName", f"Ship_{mmsi}")
            
            if lat is None or lon is None:
                return
            
            # Store vessel data
            vessel_data = {
                "mmsi": mmsi,
                "name": ship_name,
                "latitude": lat,
                "longitude": lon,
                "heading": heading,
                "speed": speed,
                "course": course,
                "timestamp": asyncio.get_event_loop().time()
            }
            
            ais_vessels[mmsi] = vessel_data
            
            # Forward to dashboard via WebSocket
            dashboard_message = json.dumps({
                "type": "vessel_update",
                "mmsi": mmsi,
                "data": vessel_data
            })
            await broadcast_to_dashboard(dashboard_message)
            
            # Print status (throttled)
            if len(ais_vessels) % 10 == 0:
                print(f"📡 Tracking {len(ais_vessels)} vessels")
                
    except Exception as e:
        print(f"⚠️ Error in process_ais_message: {e}")

async def main():
    """Main function"""
    print("=" * 50)
    print("🚢 AIS Proxy Server")
    print("=" * 50)
    print(f"API Key: {AISSTREAM_API_KEY[:10]}...")
    print(f"Dashboard WebSocket: ws://localhost:9091")
    print("=" * 50)
    
    # Start dashboard WebSocket server
    dashboard_server = await serve(handle_dashboard_client, "localhost", 9091)
    print("✅ Dashboard WebSocket server listening on port 9091")
    
    # Start AIS connection
    await connect_to_aisstream()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Shutting down AIS proxy...")
        sys.exit(0)

