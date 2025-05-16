from robot_control import app

# Static Raspberry Pi IP address
RASPBERRY_PI_IP = '192.168.101.252'

if __name__ == '__main__':
    print(f"Server running at: http://{RASPBERRY_PI_IP}:5000")
    # Run the server on the Raspberry Pi's IP
    app.run(host=RASPBERRY_PI_IP, port=5000, debug=True)