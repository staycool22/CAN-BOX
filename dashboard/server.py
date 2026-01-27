import asyncio
import json
import threading
import socket
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi import Request
import uvicorn
import os

app = FastAPI()

# Store latest vehicle state
vehicle_state = {
    "speed": 0.0,
    "target_speed": 0.0,
    "steer_angle": 0.0,
    "motors": {}
}

# UDP Listener
UDP_IP = "0.0.0.0"
UDP_PORT = 8080

def udp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((UDP_IP, UDP_PORT))
        print(f"UDP Server listening on {UDP_IP}:{UDP_PORT}")
        
        while True:
            data, addr = sock.recvfrom(4096)
            try:
                message = json.loads(data.decode('utf-8'))
                # Update global state
                global vehicle_state
                vehicle_state.update(message)
            except Exception as e:
                print(f"UDP Error: {e}")
    except OSError as e:
        print(f"UDP Bind Error: {e}. Port {UDP_PORT} might be in use.")

# Start UDP thread
udp_thread = threading.Thread(target=udp_server, daemon=True)
udp_thread.start()

# Ensure template directory exists
template_dir = os.path.join(os.path.dirname(__file__), "templates")
if not os.path.exists(template_dir):
    os.makedirs(template_dir)

templates = Jinja2Templates(directory=template_dir)

@app.get("/", response_class=HTMLResponse)
async def get(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Send state every 50ms (20Hz)
            await websocket.send_json(vehicle_state)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
